#include "inverse_kinematics.hpp"
#include "svg_plotter.hpp"
#include "trajectory_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

struct NamedTarget {
    std::string name;
    robot::JointVector referenceJoints;
};

double maximumAbsolute(const robot::JointVector& values) {
    double maximum = 0.0;
    for (double value : values) {
        maximum = std::max(maximum, std::abs(value));
    }
    return maximum;
}

void writeIkResults(const std::vector<std::string>& names,
                    const std::vector<robot::IkResult>& results,
                    const std::filesystem::path& path) {
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("failed to write IK result file");
    }
    output << "target,converged,iterations,position_error_m,orientation_error_rad,status\n";
    output << std::fixed << std::setprecision(9);
    for (std::size_t index = 0; index < results.size(); ++index) {
        output << names[index] << ',' << (results[index].converged ? 1 : 0) << ','
               << results[index].iterations << ',' << results[index].positionError
               << ',' << results[index].orientationError << ','
               << results[index].status << '\n';
    }
}

void writeTrajectory(const std::vector<robot::TrajectorySample>& trajectory,
                     const std::filesystem::path& path) {
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("failed to write trajectory file");
    }
    output << "time_s";
    for (std::size_t joint = 0; joint < robot::kDof; ++joint) {
        output << ",q" << joint + 1 << "_rad";
    }
    for (std::size_t joint = 0; joint < robot::kDof; ++joint) {
        output << ",dq" << joint + 1 << "_rad_s";
    }
    for (std::size_t joint = 0; joint < robot::kDof; ++joint) {
        output << ",ddq" << joint + 1 << "_rad_s2";
    }
    output << ",ee_x_m,ee_y_m,ee_z_m\n";
    output << std::fixed << std::setprecision(8);

    for (const auto& sample : trajectory) {
        output << sample.time;
        for (double value : sample.position) {
            output << ',' << value;
        }
        for (double value : sample.velocity) {
            output << ',' << value;
        }
        for (double value : sample.acceleration) {
            output << ',' << value;
        }
        output << ',' << sample.endEffector.position.x << ','
               << sample.endEffector.position.y << ','
               << sample.endEffector.position.z << '\n';
    }
}

}  // namespace

int main() {
    try {
        const robot::RobotModel arm = robot::RobotModel::educationalSixAxisArm();
        const robot::JointVector home{{0.0, -0.55, 1.00, 0.0, -0.45, 0.0}};

        const std::vector<NamedTarget> targets{
            {"pre_grasp", {{0.40, -0.70, 1.15, 0.18, -0.45, 0.10}}},
            {"grasp", {{0.40, -0.90, 1.35, 0.18, -0.45, 0.10}}},
            {"lift", {{0.40, -0.65, 1.05, 0.18, -0.45, 0.10}}},
            {"pre_place", {{-0.50, -0.70, 1.10, -0.10, -0.40, -0.15}}},
            {"place", {{-0.50, -0.90, 1.35, -0.10, -0.40, -0.15}}},
            {"retreat", {{-0.50, -0.65, 1.05, -0.10, -0.40, -0.15}}},
            {"home", home},
        };

        robot::IkOptions ikOptions{};
        robot::InverseKinematicsSolver solver(ikOptions);
        std::vector<robot::IkResult> ikResults;
        std::vector<std::string> targetNames;
        std::vector<robot::JointVector> trajectoryWaypoints{home};
        robot::JointVector seed = home;

        std::cout << "Robot: " << arm.name() << "\n";
        std::cout << "Solving pick-and-place targets with damped least-squares IK\n";

        for (const auto& target : targets) {
            const robot::Pose targetPose = arm.forward(target.referenceJoints);
            robot::IkResult result = solver.solve(arm, targetPose, seed);
            ikResults.push_back(result);
            targetNames.push_back(target.name);

            std::cout << std::left << std::setw(12) << target.name << "  "
                      << std::setw(10)
                      << (result.converged ? "converged" : "failed")
                      << "  iterations=" << std::setw(4) << result.iterations
                      << "  position_error=" << std::scientific
                      << result.positionError << " m"
                      << "  orientation_error=" << result.orientationError
                      << " rad\n";

            if (!result.converged) {
                std::cerr << "IK failed for target " << target.name << ": "
                          << result.status << "\n";
                return 2;
            }

            seed = result.joints;
            trajectoryWaypoints.push_back(result.joints);
        }

        const std::vector<double> segmentDurations(
            trajectoryWaypoints.size() - 1, 2.5);
        robot::QuinticTrajectoryPlanner planner(0.02);
        const auto trajectory =
            planner.planWaypoints(arm, trajectoryWaypoints, segmentDurations);

        std::filesystem::create_directories("results");
        std::filesystem::create_directories("media");
        writeIkResults(targetNames, ikResults, "results/ik_results.csv");
        writeTrajectory(trajectory, "results/trajectory.csv");

        std::vector<double> targetTimes;
        double cumulativeTime = 0.0;
        for (double duration : segmentDurations) {
            cumulativeTime += duration;
            targetTimes.push_back(cumulativeTime);
        }
        robot::writeTrajectorySvg(
            trajectory, targetTimes, "results/joint_trajectory.svg");

        double maximumVelocity = 0.0;
        double maximumAcceleration = 0.0;
        for (const auto& sample : trajectory) {
            maximumVelocity =
                std::max(maximumVelocity, maximumAbsolute(sample.velocity));
            maximumAcceleration =
                std::max(maximumAcceleration, maximumAbsolute(sample.acceleration));
        }

        const double meanIterations = std::accumulate(
            ikResults.begin(),
            ikResults.end(),
            0.0,
            [](double total, const robot::IkResult& result) {
                return total + static_cast<double>(result.iterations);
            }) / static_cast<double>(ikResults.size());

        const auto maximumPositionError = std::max_element(
            ikResults.begin(),
            ikResults.end(),
            [](const robot::IkResult& lhs, const robot::IkResult& rhs) {
                return lhs.positionError < rhs.positionError;
            })->positionError;
        const auto maximumOrientationError = std::max_element(
            ikResults.begin(),
            ikResults.end(),
            [](const robot::IkResult& lhs, const robot::IkResult& rhs) {
                return lhs.orientationError < rhs.orientationError;
            })->orientationError;

        const robot::Pose finalPose = arm.forward(trajectory.back().position);
        const robot::Pose expectedFinalPose = arm.forward(home);
        const double finalPositionError =
            robot::positionError(finalPose, expectedFinalPose);
        const double finalOrientationError =
            robot::orientationError(finalPose, expectedFinalPose);

        std::ofstream metrics("results/summary_metrics.csv");
        metrics << "metric,value,unit\n" << std::fixed << std::setprecision(9)
                << "ik_targets," << ikResults.size() << ",count\n"
                << "ik_successes," << ikResults.size() << ",count\n"
                << "mean_ik_iterations," << meanIterations << ",iterations\n"
                << "maximum_ik_position_error," << maximumPositionError << ",m\n"
                << "maximum_ik_orientation_error," << maximumOrientationError
                << ",rad\n"
                << "trajectory_duration," << trajectory.back().time << ",s\n"
                << "trajectory_samples," << trajectory.size() << ",count\n"
                << "maximum_joint_velocity," << maximumVelocity << ",rad/s\n"
                << "maximum_joint_acceleration," << maximumAcceleration
                << ",rad/s^2\n"
                << "final_position_error," << finalPositionError << ",m\n"
                << "final_orientation_error," << finalOrientationError
                << ",rad\n";

        std::cout << std::fixed << std::setprecision(6)
                  << "\nTrajectory duration: " << trajectory.back().time << " s\n"
                  << "Trajectory samples: " << trajectory.size() << "\n"
                  << "Maximum joint velocity: " << maximumVelocity << " rad/s\n"
                  << "Maximum joint acceleration: " << maximumAcceleration
                  << " rad/s^2\n"
                  << "Final position error: " << finalPositionError << " m\n"
                  << "Final orientation error: " << finalOrientationError
                  << " rad\n"
                  << "Outputs written to results/\n";
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "Error: " << exception.what() << '\n';
        return 1;
    }
}
