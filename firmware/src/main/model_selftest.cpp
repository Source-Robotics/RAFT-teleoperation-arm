#include "model_selftest.h"

#include <math.h>

#include "dynamics.h"
#include "golden_vectors.h"
#include "kinematics.h"

namespace raft {

ModelSelfTestResult runModelSelfTest()
{
    ModelSelfTestResult r = {0, 0.0f, 0.0f, 0.0f, 0.0f, false};
    for (int c = 0; c < raft_model::NUM_GOLDEN; ++c) {
        const raft_model::GoldenCase& g = raft_model::GOLDEN[c];
        Frame T[kNumFrames];
        forwardKinematics(g.q, T);
        float tau[kNumJoints];
        gravityTorque(T, tau);

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                const float e = fabsf(T[6].R[3 * i + j] - g.T06[4 * i + j]);
                if (e > r.maxRotErr) r.maxRotErr = e;
            }
            const float e = fabsf(T[6].p[i] - g.T06[4 * i + 3]);
            if (e > r.maxPosErr) r.maxPosErr = e;
        }
        for (int i = 0; i < kNumJoints; ++i) {
            const float e = fabsf(tau[i] - g.tau_g[i]);
            if (e > r.maxTauErr) r.maxTauErr = e;
        }
        if (fabsf(g.T06[2]) < 0.999f) {                // roll and yaw are ill-defined near pitch = +-90 deg
            float rpy[3];
            rpyXyz(T[6].R, rpy);
            for (int i = 0; i < 3; ++i) {
                const float e = fabsf(wrapPi(rpy[i] - g.rpy[i]));
                if (e > r.maxRpyErr) r.maxRpyErr = e;
            }
        }
        ++r.cases;
    }
    r.pass = r.maxPosErr < kSelfTestPosTol && r.maxRotErr < kSelfTestRotTol &&
             r.maxTauErr < kSelfTestTauTol && r.maxRpyErr < kSelfTestRpyTol;
    return r;
}

}  // namespace raft
