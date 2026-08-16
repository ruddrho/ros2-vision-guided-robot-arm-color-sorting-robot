#pragma once

#include "robot_model.hpp"

#include <cstddef>
#include <string>

namespace robot {

struct IkOptions {
    std::size_t maximumIterations{700};
    double positionTolerance{1e-4};
    double orientationTolerance{1e-3};
    double damping{0.06};
    double orientationWeight{0.45};
    double stepScale{0.70};
    double maximumJointStep{0.16};
    std::size_t stagnationIterations{80};
    double stagnationTolerance{1e-10};
};

struct IkResult {
    bool converged{false};
    JointVector joints{};
    std::size_t iterations{0};
    double positionError{0.0};
    double orientationError{0.0};
    std::string status;
};

class InverseKinematicsSolver {
public:
    explicit InverseKinematicsSolver(IkOptions options = {});

    IkResult solve(const RobotModel& robot,
                   const Pose& target,
                   const JointVector& initialGuess) const;

    const IkOptions& options() const;

private:
    IkOptions options_;
};

}  // namespace robot
