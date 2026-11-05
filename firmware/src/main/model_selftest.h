// Boot self-test of the on-device model against golden_vectors.h (generated offline by tools/gen_model.py).
// The firmware must not enable torque when this fails.
#pragma once

namespace raft {

struct ModelSelfTestResult {
    int   cases;
    float maxPosErr;   // m, TCP position
    float maxRotErr;   // TCP rotation matrix entries
    float maxTauErr;   // Nm, gravity torque
    float maxRpyErr;   // rad, TCP roll/pitch/yaw (cases away from the pitch = +-90 deg singularity)
    bool  pass;
};

constexpr float kSelfTestPosTol = 1e-5f;
constexpr float kSelfTestRotTol = 1e-5f;
constexpr float kSelfTestTauTol = 1e-4f;
constexpr float kSelfTestRpyTol = 1e-3f;

ModelSelfTestResult runModelSelfTest();

}  // namespace raft
