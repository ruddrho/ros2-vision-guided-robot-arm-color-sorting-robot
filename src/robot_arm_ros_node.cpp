#include "inverse_kinematics.hpp"
#include "trajectory_planner.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

struct NamedTarget {
    std::string name;
    robot::JointVector referenceJoints;
};

geometry_msgs::msg::Quaternion rotationToQuaternion(const robot::Mat3& rotation) {
    geometry_msgs::msg::Quaternion quaternion;
    const double trace = rotation[0][0] + rotation[1][1] + rotation[2][2];

    if (trace > 0.0) {
        const double scale = 2.0 * std::sqrt(trace + 1.0);
        quaternion.w = 0.25 * scale;
        quaternion.x = (rotation[2][1] - rotation[1][2]) / scale;
        quaternion.y = (rotation[0][2] - rotation[2][0]) / scale;
        quaternion.z = (rotation[1][0] - rotation[0][1]) / scale;
    } else if (rotation[0][0] > rotation[1][1] &&
               rotation[0][0] > rotation[2][2]) {
        const double scale =
            2.0 * std::sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]);
        quaternion.w = (rotation[2][1] - rotation[1][2]) / scale;
        quaternion.x = 0.25 * scale;
        quaternion.y = (rotation[0][1] + rotation[1][0]) / scale;
        quaternion.z = (rotation[0][2] + rotation[2][0]) / scale;
    } else if (rotation[1][1] > rotation[2][2]) {
        const double scale =
            2.0 * std::sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]);
        quaternion.w = (rotation[0][2] - rotation[2][0]) / scale;
        quaternion.x = (rotation[0][1] + rotation[1][0]) / scale;
        quaternion.y = 0.25 * scale;
        quaternion.z = (rotation[1][2] + rotation[2][1]) / scale;
    } else {
        const double scale =
            2.0 * std::sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]);
        quaternion.w = (rotation[1][0] - rotation[0][1]) / scale;
        quaternion.x = (rotation[0][2] + rotation[2][0]) / scale;
        quaternion.y = (rotation[1][2] + rotation[2][1]) / scale;
        quaternion.z = 0.25 * scale;
    }
    return quaternion;
}

geometry_msgs::msg::Pose poseMessage(const robot::Pose& pose) {
    geometry_msgs::msg::Pose message;
    message.position.x = pose.position.x;
    message.position.y = pose.position.y;
    message.position.z = pose.position.z;
    message.orientation = rotationToQuaternion(pose.rotation);
    return message;
}

class RobotArmRosNode : public rclcpp::Node {
public:
    RobotArmRosNode()
        : Node("robot_arm_trajectory_player"),
          arm_(robot::RobotModel::educationalSixAxisArm()) {
        const double playbackRate = declare_parameter<double>("playback_rate", 1.0);
        loop_ = declare_parameter<bool>("loop", true);
        if (playbackRate <= 0.0) {
            throw std::invalid_argument("playback_rate must be positive");
        }

        trajectory_ = generateTrajectory();
        if (trajectory_.empty()) {
            throw std::runtime_error("trajectory generation returned no samples");
        }

        jointPublisher_ =
            create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        posePublisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "end_effector_pose", 10);
        pathPublisher_ = create_publisher<nav_msgs::msg::Path>(
            "planned_path", rclcpp::QoS(1).transient_local().reliable());

        publishPlannedPath();

        const double timerPeriodSeconds = plannerSampleTime_ / playbackRate;
        const auto timerPeriod = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(timerPeriodSeconds));
        timer_ = create_wall_timer(timerPeriod, [this]() { publishNextSample(); });

        RCLCPP_INFO(get_logger(),
                    "Generated %zu samples for a %.2f second trajectory",
                    trajectory_.size(),
                    trajectory_.back().time);
        RCLCPP_INFO(get_logger(),
                    "Publishing joint_states, end_effector_pose, and planned_path");
    }

private:
    std::vector<robot::TrajectorySample> generateTrajectory() const {
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

        const robot::InverseKinematicsSolver solver(robot::IkOptions{});
        std::vector<robot::JointVector> waypoints{home};
        robot::JointVector seed = home;

        for (const auto& target : targets) {
            const robot::Pose targetPose = arm_.forward(target.referenceJoints);
            const robot::IkResult result = solver.solve(arm_, targetPose, seed);
            if (!result.converged) {
                throw std::runtime_error("IK failed for " + target.name +
                                         ": " + result.status);
            }
            waypoints.push_back(result.joints);
            seed = result.joints;
        }

        const std::vector<double> segmentDurations(waypoints.size() - 1, 2.5);
        const robot::QuinticTrajectoryPlanner planner(plannerSampleTime_);
        return planner.planWaypoints(arm_, waypoints, segmentDurations);
    }

    void publishPlannedPath() {
        nav_msgs::msg::Path path;
        path.header.frame_id = frameId_;
        path.header.stamp = now();
        path.poses.reserve(trajectory_.size());

        for (const auto& sample : trajectory_) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose = poseMessage(sample.endEffector);
            path.poses.push_back(std::move(pose));
        }
        pathPublisher_->publish(path);
    }

    void publishNextSample() {
        const auto& sample = trajectory_[sampleIndex_];
        const rclcpp::Time stamp = now();

        sensor_msgs::msg::JointState jointState;
        jointState.header.stamp = stamp;
        jointState.name.assign(jointNames_.begin(), jointNames_.end());
        jointState.position.assign(sample.position.begin(), sample.position.end());
        jointState.velocity.assign(sample.velocity.begin(), sample.velocity.end());
        jointPublisher_->publish(jointState);

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = frameId_;
        pose.pose = poseMessage(sample.endEffector);
        posePublisher_->publish(pose);

        ++sampleIndex_;
        if (sampleIndex_ >= trajectory_.size()) {
            if (loop_) {
                sampleIndex_ = 0;
            } else {
                sampleIndex_ = trajectory_.size() - 1;
                timer_->cancel();
                RCLCPP_INFO(get_logger(), "Trajectory playback completed");
            }
        }
    }

    static constexpr double plannerSampleTime_{0.02};
    const std::string frameId_{"base_link"};
    const std::array<std::string, robot::kDof> jointNames_{
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    robot::RobotModel arm_;
    std::vector<robot::TrajectorySample> trajectory_;
    std::size_t sampleIndex_{0};
    bool loop_{true};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPublisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr posePublisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<RobotArmRosNode>());
    } catch (const std::exception& exception) {
        RCLCPP_FATAL(rclcpp::get_logger("robot_arm_ros_node"),
                     "Fatal error: %s", exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
