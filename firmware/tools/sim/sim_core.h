// Shared pieces of the RAFT simulators (dev tool, nothing here runs on the arm):
//   Arm        one MuJoCo arm (RAFT.xml based scene): state, external forces, spring tendon, limits
//   ServoBus   what the HLS3606 bus does to the signals, plus a first-order motor heating model
//   Hand       a hand on the handle: stiff spring from the TCP to a target on a minimum-jerk path
//   Operator   a joint-space "operator" for long sessions (holds poses, lets go)
//   Leader     Arm + ServoBus + the firmware controller, stepped at 1 kHz with control at 250 Hz (kControlHz)
// Servo friction, armature and the thermal constants are ESTIMATES until measured on the bench.
#pragma once

#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "arm_design.h"
#include "control.h"
#include "servo_types.h"

namespace sim {

const double kPi = 3.14159265358979323846;
const double kRadPerTick = 2.0 * kPi / 4096.0;
const double kRadPsPerSpeedUnit = 0.732 * 2.0 * kPi / 60.0;
const int kCtrlEvery = 1000 / raft::kControlHz;   // 1 kHz physics / 250 Hz control, like the arm
const double kStallNm = 6.0 * 0.0980665;

inline double deg(double r) { return r * 180.0 / kPi; }
inline double rad(double d) { return d * kPi / 180.0; }
inline double dist3(const double a[3], const double b[3])
{
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}
inline double minJerk(double u) { u = u < 0 ? 0 : (u > 1 ? 1 : u); return u * u * u * (10 - 15 * u + 6 * u * u); }

class Arm {
public:
    explicit Arm(const std::string& scene)
    {
        char err[1000] = "";
        m = mj_loadXML(scene.c_str(), nullptr, err, sizeof(err));
        if (!m) { std::printf("cannot load %s: %s\n", scene.c_str(), err); return; }
        d = mj_makeData(m);
        tcpSite = mj_name2id(m, mjOBJ_SITE, "tcp");
        link6 = mj_name2id(m, mjOBJ_BODY, "link6");
        spring = mj_name2id(m, mjOBJ_TENDON, "j2_spring");
        const char* jn[7] = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_trigger_joint"};
        for (int i = 0; i < 7; ++i) {
            jnt[i] = mj_name2id(m, mjOBJ_JOINT, jn[i]);
            qadr[i] = m->jnt_qposadr[jnt[i]];
            vadr[i] = m->jnt_dofadr[jnt[i]];
        }
        for (int b = 0; b < m->nbody && b < 64; ++b) {
            mass0[b] = m->body_mass[b];
            for (int k = 0; k < 3; ++k) inertia0[b][k] = m->body_inertia[3 * b + k];
        }
        for (int i = 0; i < 7; ++i) friction0[i] = m->dof_frictionloss[vadr[i]];
    }
    ~Arm() { if (d) mj_deleteData(d); if (m) mj_deleteModel(m); }
    Arm(const Arm&) = delete;
    Arm& operator=(const Arm&) = delete;
    bool ok() const { return m && d; }

    // friction < 0 keeps the MJCF estimate; massScale scales the moving links
    void configure(double friction, double massScale)
    {
        for (int i = 0; i < 7; ++i) m->dof_frictionloss[vadr[i]] = friction < 0 ? friction0[i] : friction;
        for (int b = 1; b < m->nbody && b < 64; ++b) {
            const double s = m->body_jntnum[b] > 0 ? massScale : 1.0;
            m->body_mass[b] = mass0[b] * s;
            for (int k = 0; k < 3; ++k) m->body_inertia[3 * b + k] = inertia0[b][k] * s;
        }
        mj_setConst(m, d);
    }
    void setSpringK(double kNperM) { if (spring >= 0) m->tendon_stiffness[spring] = kNperM; }
    void setTorqueLimit(double nm) { for (int a = 0; a < m->nu; ++a) { m->actuator_ctrlrange[2 * a] = -nm; m->actuator_ctrlrange[2 * a + 1] = nm; } }
    void setJointRange(int i, double lo, double hi) { m->jnt_range[2 * jnt[i]] = lo; m->jnt_range[2 * jnt[i] + 1] = hi; m->jnt_limited[jnt[i]] = 1; }

    void reset(const double q0[7])
    {
        mj_resetData(m, d);
        for (int i = 0; i < 7; ++i) d->qpos[qadr[i]] = q0[i];
        mj_forward(m, d);
        clearApplied();
    }
    double q(int i) const { return d->qpos[qadr[i]]; }
    double qd(int i) const { return d->qvel[vadr[i]]; }
    double bias(int i) const { return d->qfrc_bias[vadr[i]]; }
    void tcp(double p[3]) const { for (int k = 0; k < 3; ++k) p[k] = d->site_xpos[3 * tcpSite + k]; }
    // set a joint by hand (kinematically, for frozen steps) and the torque left over after actuators + springs
    void setQ(int i, double v) { d->qpos[qadr[i]] = v; d->qvel[vadr[i]] = 0; }
    double residual(int i) const { return d->qfrc_actuator[vadr[i]] + d->qfrc_passive[vadr[i]] - d->qfrc_bias[vadr[i]]; }

    void clearApplied() { std::memset(d->qfrc_applied, 0, sizeof(mjtNum) * m->nv); }
    void applyTcpForce(const double f[3])
    {
        const mjtNum zero[3] = {0, 0, 0};
        mj_applyFT(m, d, f, zero, d->site_xpos + 3 * tcpSite, link6, d->qfrc_applied);
    }
    void applyJointTorque(int i, double tau) { d->qfrc_applied[vadr[i]] += tau; }

    // one physics step with the given actuator torques; frozen = held still (only kinematics updated)
    void step(const double ctrl[7], bool frozen)
    {
        for (int i = 0; i < 7; ++i) d->ctrl[i] = ctrl[i];
        if (frozen) { mj_forward(m, d); return; }
        mj_step(m, d);
    }

    mjModel* m = nullptr;
    mjData* d = nullptr;
    int tcpSite = -1, link6 = -1, spring = -1, jnt[7] = {0}, qadr[7] = {0}, vadr[7] = {0};
    double mass0[64] = {0}, inertia0[64][3] = {{0}}, friction0[7] = {0};
};

// Motor heating, one first-order model per servo: C dT/dt = I^2 R(T) - (T - Tamb) / Rth.
// ESTIMATES: Rth chosen so the rated 350 mA continuous settles at 70 C in 25 C air (131 K/W), time
// constant 10 min. The real numbers come from the P3 temperature logs.
struct Thermal {
    double R25 = 2.8, alpha = 0.0039, Rth = 131.0, tauS = 600.0, Tamb = 25.0;
    double T[7] = {25, 25, 25, 25, 25, 25, 25};
    void reset(double T0) { for (double& t : T) t = T0; }
    void update(int i, double amps, double dt)
    {
        const double R = R25 * (1.0 + alpha * (T[i] - 25.0));
        const double C = tauS / Rth;
        T[i] += dt * (amps * amps * R - (T[i] - Tamb) / Rth) / C;
    }
};

struct Truth {                      // the real servo, which the firmware only estimates
    double mAPerUnit = 3.25;      // the HLS3606's current register unit (as on the arm)
    double ktNmPerA = 4.5 * 0.0980665;
    // raw-level truth (used by readRaw and apply): encoder zero, sign, per-joint Kt error, current register scale
    int    zeroTicks[7] = {2048, 2048, 2048, 2048, 2048, 2048, 2048};
    int    dir[7] = {1, 1, 1, 1, 1, 1, 1};
    int    currentSign = -1;        // the HLS3606: a positive goal current turns it toward lower counts (as on the arm)
    double ktScale[7] = {1, 1, 1, 1, 1, 1, 1};
    double regPerUnit = 1.0;
};

class ServoBus {
public:
    Truth truth;
    Thermal thermal;
    int latencySteps = 2;           // command lands 2 ms after the read

    void reset() { for (int i = 0; i < 7; ++i) { pending[i] = active[i] = 0; } pendingAt = -1; }
    void read(const Arm& a, raft::ControlInput& in) const
    {
        for (int i = 0; i < 7; ++i) {
            in.q[i] = (float)(std::lround(a.q(i) / kRadPerTick) * kRadPerTick);
            in.qd[i] = (float)(std::lround(a.qd(i) / kRadPsPerSpeedUnit) * kRadPsPerSpeedUnit);
            in.tempC[i] = (float)std::floor(thermal.T[i]);      // the servo reports whole degrees
        }
    }
    // What the real bus reports: raw ticks with the servo's own zero and sign, speed units, present current.
    void readRaw(const Arm& a, raft::RawServoState& r) const
    {
        for (int i = 0; i < 7; ++i) {
            r.ok[i] = true;
            int t = (truth.zeroTicks[i] + truth.dir[i] * (int)std::lround(a.q(i) / kRadPerTick)) % 4096;
            if (t < 0) t += 4096;
            r.pos[i] = (int16_t)t;
            r.speed[i] = (int16_t)(truth.dir[i] * std::lround(a.qd(i) / kRadPsPerSpeedUnit));
            r.current[i] = (int16_t)std::lround(active[i] * truth.regPerUnit);
            r.tempC[i] = (uint8_t)std::floor(thermal.T[i]);
            r.voltage[i] = 60;
        }
    }
    void command(const int16_t raw[7], long step) { for (int i = 0; i < 7; ++i) pending[i] = raw[i]; pendingAt = step + latencySteps; }
    void apply(long step, double dt, double ctrl[7])
    {
        if (step == pendingAt) for (int i = 0; i < 7; ++i) if (!(selfMask & (1u << i))) active[i] = pending[i];
        for (int i = 0; i < 7; ++i) {
            const double amps = active[i] * truth.mAPerUnit * 1e-3;
            ctrl[i] = std::fmax(-kStallNm, std::fmin(kStallNm, truth.currentSign * truth.dir[i] * amps * truth.ktNmPerA * truth.ktScale[i]));
            thermal.update(i, amps, dt);
            current[i] = std::fabs(amps);
        }
    }
    double current[7] = {0};
    // Servos that regulate themselves (web_sim's position mode): bit i of selfMask = servo i ignores the commanded
    // current and takes setActive's, at once, without the bus latency.
    uint8_t selfMask = 0;
    void setActive(int i, int raw) { active[i] = raw; }
private:
    int pending[7] = {0}, active[7] = {0};
    long pendingAt = -1;
};

struct Hand {
    double t0 = 0, t1 = 0, tRelease = -1;     // move from p0 to p0+delta between t0 and t1, let go at tRelease
    double p0[3] = {0, 0, 0}, delta[3] = {0, 0, 0};
    double k = 400.0, c = 8.0;                 // N/m, N s/m
    double peak = 0, sum = 0; long n = 0;      // force statistics while moving (t0..t1)
    double last = 0;                           // force magnitude of the last step
    double prev[3] = {0, 0, 0}; bool havePrev = false;
    // finger on the trigger: joint spring toward fingerQ (rad) while fingerOn
    bool fingerOn = false; double fingerQ = 0, fingerK = 0.5, fingerD = 0.01, fingerTau = 0;
    void target(double t, double p[3]) const { const double s = minJerk((t - t0) / (t1 - t0)); for (int k2 = 0; k2 < 3; ++k2) p[k2] = p0[k2] + s * delta[k2]; }
};

inline void applyHand(Arm& a, Hand& h, double t, double dt)
{
    double p[3]; a.tcp(p);
    if (h.tRelease < 0 || t < h.tRelease) {
        double tg[3]; h.target(t, tg);
        double f[3], fn = 0;
        for (int k = 0; k < 3; ++k) {
            const double v = h.havePrev ? (p[k] - h.prev[k]) / dt : 0.0;
            f[k] = h.k * (tg[k] - p[k]) - h.c * v;
            fn += f[k] * f[k];
        }
        fn = std::sqrt(fn);
        h.last = fn;
        if (t >= h.t0 && t <= h.t1) { h.peak = std::fmax(h.peak, fn); h.sum += fn; ++h.n; }
        a.applyTcpForce(f);
    } else {
        h.last = 0;
    }
    if (h.fingerOn) {
        h.fingerTau = h.fingerK * (h.fingerQ - a.q(6)) - h.fingerD * a.qd(6);
        a.applyJointTorque(6, h.fingerTau);
    }
    for (int k = 0; k < 3; ++k) h.prev[k] = p[k];
    h.havePrev = true;
}

struct Operator {                     // soft joint-space PD standing in for a person holding the handle
    bool active = false;
    double q[7] = {0}, K = 1.5, D = 0.05;
    double lastTau[7] = {0};
};

inline void applyOperator(Arm& a, Operator& op)
{
    for (int i = 0; i < 7; ++i) {
        op.lastTau[i] = op.active ? op.K * (op.q[i] - a.q(i)) - op.D * a.qd(i) : 0.0;
        if (op.active) a.applyJointTorque(i, op.lastTau[i]);
    }
}

// The leader: MuJoCo arm + simulated bus + the firmware controller.
class Leader {
public:
    explicit Leader(const std::string& scene) : arm(scene) {}
    Arm arm;
    ServoBus bus;
    long step = 0;
    raft::ControlInput in = {};
    raft::ControlOutput out = {};

    double dt() const { return arm.m->opt.timestep; }
    double time() const { return step * dt(); }
    void reset(const double q0[7], double T0 = 25.0) { arm.reset(q0); bus.reset(); bus.thermal.reset(T0); step = 0; }

    // One physics step. On control ticks: pre(in) -> Controller::step -> post(in, out).
    template <class Pre, class Post>
    bool stepOnce(raft::Controller& ctl, Hand* hand, Operator* op, bool frozen, Pre pre, Post post)
    {
        bool ctrlTick = false;
        if (step % kCtrlEvery == 0) {
            bus.read(arm, in);
            in.followerFresh = false;
            pre(in);
            ctl.step(in, (float)(kCtrlEvery * dt()), out);
            post(in, out);
            bus.command(out.raw, step);
            ctrlTick = true;
        }
        double ctrl[7];
        bus.apply(step, dt(), ctrl);
        arm.clearApplied();
        if (!frozen) {
            if (hand) applyHand(arm, *hand, time(), dt());
            if (op) applyOperator(arm, *op);
        }
        arm.step(ctrl, frozen);
        ++step;
        return ctrlTick;
    }
    bool stepOnce(raft::Controller& ctl, Hand* hand = nullptr, Operator* op = nullptr, bool frozen = false)
    {
        return stepOnce(ctl, hand, op, frozen, [](raft::ControlInput&) {}, [](const raft::ControlInput&, const raft::ControlOutput&) {});
    }
    void run(double seconds, raft::Controller& ctl, Hand* hand = nullptr, Operator* op = nullptr, bool frozen = false)
    {
        const long n = std::lround(seconds / dt());
        for (long k = 0; k < n; ++k) stepOnce(ctl, hand, op, frozen);
    }
};

inline std::string defaultScene(const char* argv0, const char* file)
{
    std::string exe = argv0;
    const size_t cut = exe.find_last_of("/\\");
    const std::string dir = (cut == std::string::npos) ? "." : exe.substr(0, cut);
    return dir + "/../model/" + file;     // tools/sim/out -> tools/sim/model (written by tools/gen_model.py)
}

}  // namespace sim
