#include "inverse_kinematics.hpp"
#include "trajectory_planner.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testForwardKinematicsIsFinite() {
    const auto robot = robot::RobotModel::educationalSixAxisArm();
    const robot::JointVector joints{{0.1, -0.4, 0.8, 0.2, -0.3, 0.5}};
    const auto transform = robot.forwardTransform(joints);
    for (const auto& row : transform.data) {
        for (double value : row) {
            require(std::isfinite(value), "forward transform contains non-finite value");
        }
    }
    require(std::abs(transform.data[3][0]) < 1e-12 &&
                std::abs(transform.data[3][1]) < 1e-12 &&
                std::abs(transform.data[3][2]) < 1e-12 &&
                std::abs(transform.data[3][3] - 1.0) < 1e-12,
            "homogeneous transform bottom row is invalid");
}

void testJacobianTranslationAgainstFiniteDifference() {
    const auto arm = robot::RobotModel::educationalSixAxisArm();
    const robot::JointVector joints{{0.2, -0.7, 1.0, 0.25, -0.5, 0.3}};
    const auto jacobian = arm.geometricJacobian(joints);
    const auto basePose = arm.forward(joints);
    constexpr double epsilon = 1e-7;

    for (std::size_t joint = 0; joint < robot::kDof; ++joint) {
        auto perturbed = joints;
        perturbed[joint] += epsilon;
        const auto pose = arm.forward(perturbed);
        const robot::Vec3 numerical =
            (pose.position - basePose.position) / epsilon;
        require(std::abs(numerical.x - jacobian[0][joint]) < 2e-5 &&
                    std::abs(numerical.y - jacobian[1][joint]) < 2e-5 &&
                    std::abs(numerical.z - jacobian[2][joint]) < 2e-5,
                "analytical Jacobian does not match finite difference");
    }
}

void testInverseKinematicsReachablePose() {
    const auto arm = robot::RobotModel::educationalSixAxisArm();
    const robot::JointVector targetJoints{{0.35, -0.75, 1.20, 0.15, -0.45, 0.12}};
    const robot::JointVector seed{{0.0, -0.55, 1.0, 0.0, -0.45, 0.0}};
    const robot::Pose target = arm.forward(targetJoints);
    const robot::InverseKinematicsSolver solver;
    const auto result = solver.solve(arm, target, seed);
    require(result.converged, "IK did not converge for a reachable target");
    require(result.positionError <= solver.options().positionTolerance,
            "IK position tolerance was not achieved");
    require(result.orientationError <= solver.options().orientationTolerance,
            "IK orientation tolerance was not achieved");
    require(arm.withinLimits(result.joints), "IK result violates joint limits");
}

void testQuinticTrajectoryBoundaryConditions() {
    const auto arm = robot::RobotModel::educationalSixAxisArm();
    const robot::JointVector start{{0.0, -0.5, 1.0, 0.0, -0.4, 0.0}};
    const robot::JointVector goal{{0.3, -0.8, 1.2, 0.2, -0.5, 0.1}};
    const robot::QuinticTrajectoryPlanner planner(0.01);
    const auto trajectory = planner.planSegment(arm, start, goal, 2.0);
    require(!trajectory.empty(), "trajectory is empty");
    for (std::size_t joint = 0; joint < robot::kDof; ++joint) {
        require(std::abs(trajectory.front().position[joint] - start[joint]) < 1e-12,
                "trajectory start position is incorrect");
        require(std::abs(trajectory.back().position[joint] - goal[joint]) < 1e-12,
                "trajectory goal position is incorrect");
        require(std::abs(trajectory.front().velocity[joint]) < 1e-12 &&
                    std::abs(trajectory.back().velocity[joint]) < 1e-12,
                "trajectory endpoint velocity is not zero");
        require(std::abs(trajectory.front().acceleration[joint]) < 1e-12 &&
                    std::abs(trajectory.back().acceleration[joint]) < 1e-12,
                "trajectory endpoint acceleration is not zero");
    }
}

void testJointLimitClamping() {
    const auto arm = robot::RobotModel::educationalSixAxisArm();
    const robot::JointVector outside{{8.0, -8.0, 6.0, -7.0, 7.0, -9.0}};
    const auto clamped = arm.clampToLimits(outside);
    require(arm.withinLimits(clamped), "joint clamping failed");
}

}  // namespace

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"forward kinematics", testForwardKinematicsIsFinite},
        {"Jacobian finite difference", testJacobianTranslationAgainstFiniteDifference},
        {"inverse kinematics", testInverseKinematicsReachablePose},
        {"quintic trajectory", testQuinticTrajectoryBoundaryConditions},
        {"joint limits", testJointLimitClamping},
    };

    std::size_t failures = 0;
    for (const auto& [name, test] : tests) {
        try {
            test();
            std::cout << "[PASS] " << name << '\n';
        } catch (const std::exception& exception) {
            ++failures;
            std::cerr << "[FAIL] " << name << ": " << exception.what() << '\n';
        }
    }

    std::cout << tests.size() - failures << '/' << tests.size()
              << " tests passed\n";
    return failures == 0 ? 0 : 1;
}
