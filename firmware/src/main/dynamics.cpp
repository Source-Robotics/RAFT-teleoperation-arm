#include "dynamics.h"

#include "raft_model.h"

namespace raft {

// Static recursive Newton-Euler, written in the base frame. Walk from the trigger back to the base,
// accumulating the total gravity force f of everything distal and its moment n about the base origin;
// the holding torque of joint i is minus the moment about that joint's axis:
//     tau_i = -z_i . (n - o_i x f)
// Gravity is along -z of the base, so f = (0, 0, fz) and the z component of n stays zero.
void gravityTorque(const Frame T[kNumFrames], float tau[kNumJoints])
{
    float fz = 0.0f;           // total distal gravity force, z component (N)
    float nx = 0.0f, ny = 0.0f; // its moment about the base origin (Nm)

    for (int i = kNumJoints - 1; i >= 0; --i) {
        const Frame& L = T[i + 1];                     // link i's frame (link 6 = trigger -> hinge frame)
        const raft_model::LinkInertial& li = raft_model::LINKS[i];
        const float cx = L.R[0] * li.r[0] + L.R[1] * li.r[1] + L.R[2] * li.r[2] + L.p[0];
        const float cy = L.R[3] * li.r[0] + L.R[4] * li.r[1] + L.R[5] * li.r[2] + L.p[1];
        const float w = -li.m * raft_model::GRAVITY;
        fz += w;
        nx += cy * w;           // c x (0, 0, w) = (cy w, -cx w, 0)
        ny -= cx * w;

        // joint i turns about z of its parent frame: DH frame i-1 for the arm, the hinge frame for the trigger
        const Frame& J = (i < raft_model::NUM_ARM_JOINTS) ? T[i] : T[7];
        const float zx = J.R[2], zy = J.R[5];          // z column; its z component meets a zero moment
        const float mx = nx - J.p[1] * fz;             // n - o x f, with o x (0, 0, fz) = (oy fz, -ox fz, 0)
        const float my = ny + J.p[0] * fz;
        tau[i] = -(zx * mx + zy * my);
    }
}

void gravityTorque(const float q[kNumJoints], float tau[kNumJoints])
{
    Frame T[kNumFrames];
    forwardKinematics(q, T);
    gravityTorque(T, tau);
}

}  // namespace raft
