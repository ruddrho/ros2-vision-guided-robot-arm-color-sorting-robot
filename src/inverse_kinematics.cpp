#include "inverse_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot {

InverseKinematicsSolver::InverseKinematicsSolver(IkOptions options)
    : options_(options) {}

IkResult InverseKinematicsSolver::solve(const RobotModel& robot,
                                        const Pose& target,
                                        const JointVector& initialGuess) const {
    JointVector joints = robot.clampToLimits(initialGuess);
    double bestCombinedError = std::numeric_limits<double>::infinity();
    std::size_t stagnantIterations = 0;

    IkResult result{};
    result.joints = joints;

    for (std::size_t iteration = 0; iteration < options_.maximumIterations;
         ++iteration) {
        const Pose current = robot.forward(joints);
        Vector6 error = poseError(current, target);
        const double currentPositionError = robot::positionError(current, target);
        const double currentOrientationError =
            robot::orientationError(current, target);

        result.iterations = iteration;
        result.positionError = currentPositionError;
        result.orientationError = currentOrientationError;
        result.joints = joints;

        if (currentPositionError <= options_.positionTolerance &&
            currentOrientationError <= options_.orientationTolerance) {
            result.converged = true;
            result.status = "converged";
            return result;
        }

        const double combinedError =
            currentPositionError + options_.orientationWeight * currentOrientationError;
        if (bestCombinedError - combinedError > options_.stagnationTolerance) {
            bestCombinedError = combinedError;
            stagnantIterations = 0;
        } else {
            ++stagnantIterations;
        }

        if (stagnantIterations >= options_.stagnationIterations) {
            result.status = "stagnated before reaching tolerance";
            return result;
        }

        Jacobian weightedJacobian = robot.geometricJacobian(joints);
        for (std::size_t row = 3; row < 6; ++row) {
            error[row] *= options_.orientationWeight;
            for (double& value : weightedJacobian[row]) {
                value *= options_.orientationWeight;
            }
        }

        Matrix6 normalMatrix{};
        for (std::size_t row = 0; row < 6; ++row) {
            for (std::size_t column = 0; column < 6; ++column) {
                for (std::size_t joint = 0; joint < kDof; ++joint) {
                    normalMatrix[row][column] +=
                        weightedJacobian[row][joint] *
                        weightedJacobian[column][joint];
                }
            }
            normalMatrix[row][row] += options_.damping * options_.damping;
        }

        Vector6 intermediate{};
        if (!solveLinearSystem(normalMatrix, error, intermediate)) {
            result.status = "damped least-squares system was singular";
            return result;
        }

        JointVector jointUpdate{};
        for (std::size_t joint = 0; joint < kDof; ++joint) {
            for (std::size_t row = 0; row < 6; ++row) {
                jointUpdate[joint] +=
                    weightedJacobian[row][joint] * intermediate[row];
            }
            jointUpdate[joint] = clamp(
                options_.stepScale * jointUpdate[joint],
                -options_.maximumJointStep,
                options_.maximumJointStep);
            joints[joint] += jointUpdate[joint];
        }

        joints = robot.clampToLimits(joints);
    }

    const Pose finalPose = robot.forward(joints);
    result.joints = joints;
    result.iterations = options_.maximumIterations;
    result.positionError = robot::positionError(finalPose, target);
    result.orientationError = robot::orientationError(finalPose, target);
    result.status = "maximum iterations reached";
    return result;
}

const IkOptions& InverseKinematicsSolver::options() const {
    return options_;
}

}  // namespace robot
