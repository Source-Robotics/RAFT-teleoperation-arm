// RAFT forward kinematics: standard DH (CM6 table) with the generated model in raft_model.h.
// Plain C++11 with no Arduino dependencies, so the same file builds for the ESP32 and on a PC
// (tools/sim/build.cmd runs it against golden_vectors.h and inside the MuJoCo simulator).
#pragma once

namespace raft {

constexpr int kNumJoints = 7;   // J1..J6 + gripper trigger
constexpr int kNumFrames = 8;   // base, DH frames 1..6, trigger hinge

// Rigid transform in the base frame: row-major rotation R and translation p in metres.
struct Frame {
    float R[9];
    float p[3];
};

// q: joint angles in the CAD convention (rad). q = 0 is the CAD / PAR6 standby pose.
// out[0] = base (identity), out[i] = DH frame i for i = 1..6 (out[6] is the TCP),
// out[7] = trigger hinge frame, already rotated by q[6].
void forwardKinematics(const float q[kNumJoints], Frame out[kNumFrames]);

// Geometric Jacobian of the TCP (DH frame 6) for J1..J6, from a forward-kinematics result:
// [v; w] = J * qd with v (m/s) and w (rad/s) in the base frame. Row-major J[6][6].
void tcpJacobian(const Frame T[kNumFrames], float J[6][6]);

// Roll, pitch, yaw (rad) of a rotation in the RCB runtime's convention: SpatialMath
// tr2rpy(order='xyz'), ported from RCB utility/kinematics_backend.py::write_rpy_xyz.
// The singular branch (pitch = +-90 deg) uses a float tolerance instead of the runtime's double one.
void rpyXyz(const float R[9], float rpy[3]);

// PAR6 / RCB joint angles (DH theta) from CAD angles and back. Neither wraps: the runtime's DH angles are
// continuous (J3 and J6 are at pi in the standby pose, their ranges run past it: PAR6's J3 reaches 196 deg and
// its J6 +-229 deg in the CAD convention).
void cadToDh(const float q[6], float theta[6]);
void dhToCad(const float theta[6], float q[6]);

float wrapPi(float x);   // to (-pi, pi]; NaN for a non-finite input (never loops)

}  // namespace raft
