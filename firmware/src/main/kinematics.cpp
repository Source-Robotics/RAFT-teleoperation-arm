#include "kinematics.h"

#include <float.h>
#include <math.h>

#include "raft_model.h"

namespace raft {
namespace {

const float kPi = 3.14159265358979f;

// out = a * b
void compose(const Frame& a, const Frame& b, Frame& out)
{
    Frame r;
    for (int i = 0; i < 3; ++i) {
        const float a0 = a.R[3 * i], a1 = a.R[3 * i + 1], a2 = a.R[3 * i + 2];
        r.R[3 * i]     = a0 * b.R[0] + a1 * b.R[3] + a2 * b.R[6];
        r.R[3 * i + 1] = a0 * b.R[1] + a1 * b.R[4] + a2 * b.R[7];
        r.R[3 * i + 2] = a0 * b.R[2] + a1 * b.R[5] + a2 * b.R[8];
        r.p[i] = a0 * b.p[0] + a1 * b.p[1] + a2 * b.p[2] + a.p[i];
    }
    out = r;
}

// Standard DH: A = Rz(theta) Tz(d) Tx(a) Rx(alpha)
void dhTransform(float theta, const raft_model::DhLink& L, Frame& A)
{
    const float ct = cosf(theta), st = sinf(theta);
    const float ca = cosf(L.alpha), sa = sinf(L.alpha);
    A.R[0] = ct;   A.R[1] = -st * ca; A.R[2] = st * sa;
    A.R[3] = st;   A.R[4] = ct * ca;  A.R[5] = -ct * sa;
    A.R[6] = 0.0f; A.R[7] = sa;       A.R[8] = ca;
    A.p[0] = L.a * ct;
    A.p[1] = L.a * st;
    A.p[2] = L.d;
}

}  // namespace

float wrapPi(float x)
{
    // fmodf instead of a subtraction loop: an infinite input gives NaN here instead of never returning
    if (x > kPi || x <= -kPi) {
        x = fmodf(x + kPi, 2.0f * kPi);
        if (x <= 0.0f) x += 2.0f * kPi;
        x -= kPi;
    }
    return x;
}

void forwardKinematics(const float q[kNumJoints], Frame out[kNumFrames])
{
    Frame& base = out[0];
    for (int k = 0; k < 9; ++k) base.R[k] = (k % 4 == 0) ? 1.0f : 0.0f;
    base.p[0] = base.p[1] = base.p[2] = 0.0f;

    for (int i = 0; i < raft_model::NUM_ARM_JOINTS; ++i) {
        Frame A;
        dhTransform(q[i] + raft_model::DH[i].offset, raft_model::DH[i], A);
        compose(out[i], A, out[i + 1]);
    }

    // trigger: fixed hinge frame relative to DH frame 6, then Rz(q7)
    Frame H;
    for (int k = 0; k < 9; ++k) H.R[k] = raft_model::GRIP_R[k];
    for (int k = 0; k < 3; ++k) H.p[k] = raft_model::GRIP_P[k];
    Frame Rz;
    const float c = cosf(q[6]), s = sinf(q[6]);
    Rz.R[0] = c;    Rz.R[1] = -s;   Rz.R[2] = 0.0f;
    Rz.R[3] = s;    Rz.R[4] = c;    Rz.R[5] = 0.0f;
    Rz.R[6] = 0.0f; Rz.R[7] = 0.0f; Rz.R[8] = 1.0f;
    Rz.p[0] = Rz.p[1] = Rz.p[2] = 0.0f;
    Frame H7;
    compose(H, Rz, H7);
    compose(out[6], H7, out[7]);
}

void tcpJacobian(const Frame T[kNumFrames], float J[6][6])
{
    const float* p = T[6].p;
    for (int i = 0; i < raft_model::NUM_ARM_JOINTS; ++i) {
        // joint i+1 turns about z of DH frame i, through that frame's origin
        const float zx = T[i].R[2], zy = T[i].R[5], zz = T[i].R[8];
        const float rx = p[0] - T[i].p[0], ry = p[1] - T[i].p[1], rz = p[2] - T[i].p[2];
        J[0][i] = zy * rz - zz * ry;
        J[1][i] = zz * rx - zx * rz;
        J[2][i] = zx * ry - zy * rx;
        J[3][i] = zx;
        J[4][i] = zy;
        J[5][i] = zz;
    }
}

void rpyXyz(const float Rm[9], float rpy[3])
{
    // R(i, j) = Rm[3 * i + j]
    const float R00 = Rm[0], R01 = Rm[1], R02 = Rm[2];
    const float R10 = Rm[3], R11 = Rm[4], R12 = Rm[5];
    const float R20 = Rm[6], R21 = Rm[7], R22 = Rm[8];

    if (fabsf(fabsf(R02) - 1.0f) < 20.0f * FLT_EPSILON) {
        rpy[0] = 0.0f;
        rpy[2] = (R02 > 0.0f) ? atan2f(R21, R11) : -atan2f(R10, R20);
        rpy[1] = asinf(R02 > 1.0f ? 1.0f : (R02 < -1.0f ? -1.0f : R02));
        return;
    }
    rpy[0] = -atan2f(R01, R00);
    rpy[2] = -atan2f(R12, R22);
    const float a0 = fabsf(R00), a1 = fabsf(R01), a2 = fabsf(R12), a3 = fabsf(R22);
    if (a0 >= a1 && a0 >= a2 && a0 >= a3)
        rpy[1] = atanf(R02 * cosf(rpy[0]) / R00);
    else if (a1 >= a2 && a1 >= a3)
        rpy[1] = -atanf(R02 * sinf(rpy[0]) / R01);
    else if (a2 >= a3)
        rpy[1] = -atanf(R02 * sinf(rpy[2]) / R12);
    else
        rpy[1] = atanf(R02 * cosf(rpy[2]) / R22);
}

void cadToDh(const float q[6], float theta[6])
{
    // No wrapping: the RCB runtime works in continuous DH angles (PAR6.xml: J3 1.90..6.65 rad, J6 -0.94..7.23),
    // and J3 / J6 sit at exactly pi in the standby pose. Wrapping would turn +0.1 deg past standby into -179.9 deg.
    for (int i = 0; i < 6; ++i) theta[i] = q[i] + raft_model::DH[i].offset;
}

void dhToCad(const float theta[6], float q[6])
{
    // The exact inverse of cadToDh, no wrapping either: PAR6's J3 runs to 196 deg and its J6 to +-229 deg in the
    // CAD convention. Wrapping turned a valid J3 of 190 deg into -170 deg. Which 2 pi branch the leader uses is the
    // teleop session's choice (teleop.cpp).
    for (int i = 0; i < 6; ++i) q[i] = theta[i] - raft_model::DH[i].offset;
}

}  // namespace raft
