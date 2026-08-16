#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
using ActionClient = rclcpp_action::Client<FollowJointTrajectory>;
using ArmConfiguration = std::array<double, 6>;
using namespace std::chrono_literals;

struct TargetSelection {
    std::string color;
    geometry_msgs::msg::Pose pose;
};

class GazeboPickPlaceCoordinator : public rclcpp::Node {
public:
    GazeboPickPlaceCoordinator()
        : Node("gazebo_pick_place_coordinator") {
        armActionClient_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/arm_controller/follow_joint_trajectory");
        gripperActionClient_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/gripper_controller/follow_joint_trajectory");
        stagePublisher_ = create_publisher<std_msgs::msg::String>(
            "/pick_place_stage", rclcpp::QoS(1).transient_local().reliable());
        resultPublisher_ = create_publisher<std_msgs::msg::Bool>(
            "/pick_place_success", rclcpp::QoS(1).transient_local().reliable());
        for (const auto& entry : slots_) {
            const std::string& color = entry.first;
            attachPublishers_[color] = create_publisher<std_msgs::msg::Empty>(
                "/grasp/" + color + "/attach", 10);
            detachPublishers_[color] = create_publisher<std_msgs::msg::Empty>(
                "/grasp/" + color + "/detach", 10);
        }
        selectedPoseSubscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vision/selected_cube_pose",
            rclcpp::QoS(1).transient_local().reliable(),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
                acceptSelection(message->pose);
            });

        for (const auto& entry : slots_) {
            const std::string color = entry.first;
            cubePoseSubscribers_.push_back(
                create_subscription<geometry_msgs::msg::PoseArray>(
                    "/model/" + color + "_cube/pose", 10,
                    [this, color](const geometry_msgs::msg::PoseArray::SharedPtr message) {
                        if (message->poses.empty()) {
                            return;
                        }
                        // Gazebo's PosePublisher includes a zero-valued link pose first
                        // and the model pose last for this world configuration.
                        const geometry_msgs::msg::Pose& modelPose = message->poses.back();
                        if (modelPose.position.z <= 0.0) {
                            return;
                        }
                        std::lock_guard<std::mutex> lock(stateMutex_);
                        cubePoses_[color] = modelPose;
                    }));
        }
        worker_ = std::thread([this]() { runCommandLoop(); });
    }

    ~GazeboPickPlaceCoordinator() override {
        stopRequested_.store(true);
        if (worker_.joinable()) {
            worker_.join();
        }
    }

private:
    void acceptSelection(const geometry_msgs::msg::Pose& pose) {
        std::string nearestColor;
        double nearestDistance = std::numeric_limits<double>::infinity();
        for (const auto& entry : slots_) {
            const double distance = std::hypot(
                pose.position.x - entry.second.first,
                pose.position.y - entry.second.second);
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearestColor = entry.first;
            }
        }
        if (nearestColor.empty() || nearestDistance > 0.08) {
            RCLCPP_ERROR(get_logger(), "Vision pose did not match a calibrated cube slot");
            return;
        }
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (completedColors_.count(nearestColor) != 0U) {
            RCLCPP_WARN(
                get_logger(), "%s cube is already in its destination slot",
                nearestColor.c_str());
            return;
        }
        if (!selection_.has_value()) {
            selection_ = TargetSelection{nearestColor, pose};
            RCLCPP_INFO(
                get_logger(), "Accepted OpenCV selection: %s cube at (%.3f, %.3f)",
                nearestColor.c_str(), pose.position.x, pose.position.y);
        }
    }

    std::optional<TargetSelection> waitForSelection() const {
        publishStage("waiting_for_color_command");
        while (rclcpp::ok() && !stopRequested_.load()) {
            {
                std::lock_guard<std::mutex> lock(stateMutex_);
                if (selection_.has_value()) {
                    return selection_;
                }
            }
            std::this_thread::sleep_for(100ms);
        }
        return std::nullopt;
    }

    void clearSelection() {
        std::lock_guard<std::mutex> lock(stateMutex_);
        selection_.reset();
    }

    static trajectory_msgs::msg::JointTrajectoryPoint trajectoryPoint(
        const std::vector<double>& positions, double durationSeconds) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        const auto durationNanoseconds = static_cast<std::int64_t>(
            std::llround(durationSeconds * 1'000'000'000.0));
        point.time_from_start.sec = static_cast<std::int32_t>(
            durationNanoseconds / 1'000'000'000LL);
        point.time_from_start.nanosec = static_cast<std::uint32_t>(
            durationNanoseconds % 1'000'000'000LL);
        return point;
    }

    bool executeTrajectory(
        const ActionClient::SharedPtr& client,
        const std::vector<std::string>& jointNames,
        const std::vector<double>& positions,
        const std::string& stage,
        double durationSeconds) {
        publishStage(stage);
        FollowJointTrajectory::Goal goal;
        goal.trajectory.joint_names = jointNames;
        goal.trajectory.points.push_back(trajectoryPoint(positions, durationSeconds));

        auto goalFuture = client->async_send_goal(goal);
        if (goalFuture.wait_for(5s) != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "Timed out sending stage: %s", stage.c_str());
            return false;
        }
        const GoalHandle::SharedPtr goalHandle = goalFuture.get();
        if (!goalHandle) {
            RCLCPP_ERROR(get_logger(), "Controller rejected stage: %s", stage.c_str());
            return false;
        }
        auto resultFuture = client->async_get_result(goalHandle);
        const auto timeout = std::chrono::duration<double>(durationSeconds + 10.0);
        if (resultFuture.wait_for(timeout) != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "Timed out executing stage: %s", stage.c_str());
            return false;
        }
        const auto wrappedResult = resultFuture.get();
        if (wrappedResult.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(get_logger(), "Controller failed stage: %s", stage.c_str());
            return false;
        }
        RCLCPP_INFO(get_logger(), "Completed stage: %s", stage.c_str());
        return true;
    }

    bool moveArm(
        const std::string& stage,
        const ArmConfiguration& arm,
        double durationSeconds) {
        return executeTrajectory(
            armActionClient_, armJointNames_,
            std::vector<double>(arm.begin(), arm.end()), stage, durationSeconds);
    }

    bool moveGripper(
        const std::string& stage,
        double fingerPosition,
        double durationSeconds) {
        return executeTrajectory(
            gripperActionClient_, gripperJointNames_,
            {fingerPosition, fingerPosition}, stage, durationSeconds);
    }

    void publishJointCommand(
        const std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr>&
            publishers,
        const std::string& color,
        const std::string& action) const {
        const auto found = publishers.find(color);
        if (found == publishers.end()) {
            RCLCPP_ERROR(get_logger(), "No %s publisher for %s", action.c_str(), color.c_str());
            return;
        }
        std_msgs::msg::Empty message;
        for (int attempt = 0; attempt < 3; ++attempt) {
            found->second->publish(message);
            std::this_thread::sleep_for(100ms);
        }
        RCLCPP_INFO(get_logger(), "%s detachable grasp joint for %s cube",
            action.c_str(), color.c_str());
    }

    void attachCube(const std::string& color) const {
        publishJointCommand(attachPublishers_, color, "Attached");
    }

    void detachCube(const std::string& color) const {
        publishJointCommand(detachPublishers_, color, "Detached");
    }

    void releaseAllCubes() const {
        for (const auto& entry : slots_) {
            detachCube(entry.first);
        }
        std::this_thread::sleep_for(500ms);
    }

    static ArmConfiguration rotatedForPoint(
        const ArmConfiguration& reference,
        double referenceX,
        double referenceY,
        double targetX,
        double targetY) {
        ArmConfiguration configuration = reference;
        const double referenceAngle = std::atan2(referenceY, referenceX);
        const double targetAngle = std::atan2(targetY, targetX);
        configuration[0] += targetAngle - referenceAngle;
        return configuration;
    }

    void publishStage(const std::string& stage) const {
        std_msgs::msg::String message;
        message.data = stage;
        stagePublisher_->publish(message);
        RCLCPP_INFO(get_logger(), "Starting stage: %s", stage.c_str());
    }

    std::optional<geometry_msgs::msg::Pose> latestCubePose(
        const std::string& color) const {
        std::lock_guard<std::mutex> lock(stateMutex_);
        const auto found = cubePoses_.find(color);
        if (found == cubePoses_.end()) {
            return std::nullopt;
        }
        return found->second;
    }

    std::optional<geometry_msgs::msg::Pose> waitForCubePose(
        const std::string& color, std::chrono::seconds timeout) const {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (rclcpp::ok() && !stopRequested_.load() &&
               std::chrono::steady_clock::now() < deadline) {
            const auto pose = latestCubePose(color);
            if (pose.has_value()) {
                return pose;
            }
            std::this_thread::sleep_for(100ms);
        }
        return std::nullopt;
    }

    bool verifyLift(
        const std::string& color,
        const geometry_msgs::msg::Pose& initialPose) const {
        publishStage(color + ":verify_grasp");
        std::this_thread::sleep_for(750ms);
        const auto liftedPose = latestCubePose(color);
        if (!liftedPose.has_value()) {
            RCLCPP_ERROR(get_logger(), "Cube pose unavailable during grasp verification");
            return false;
        }
        const double liftHeight = liftedPose->position.z - initialPose.position.z;
        const double lateralMotion = std::hypot(
            liftedPose->position.x - initialPose.position.x,
            liftedPose->position.y - initialPose.position.y);
        const bool grasped = liftHeight > 0.045 && lateralMotion < 0.10;
        if (!grasped) {
            RCLCPP_ERROR(
                get_logger(),
                "GRASP VERIFICATION FAILED: cube lift=%.3f m, lateral motion=%.3f m",
                liftHeight, lateralMotion);
            return false;
        }
        RCLCPP_INFO(
            get_logger(), "Grasp verified: cube lifted %.3f m", liftHeight);
        return true;
    }

    void publishResult(bool success, const std::string& detail) const {
        std_msgs::msg::Bool result;
        result.data = success;
        resultPublisher_->publish(result);
        publishStage(success ? "completed_successfully" : "failed");
        if (success) {
            RCLCPP_INFO(get_logger(), "VISION PICK-AND-PLACE PASSED: %s", detail.c_str());
        } else {
            RCLCPP_ERROR(get_logger(), "VISION PICK-AND-PLACE FAILED: %s", detail.c_str());
        }
    }

    bool runSequence(const TargetSelection& selection) {
        const auto initialPose = waitForCubePose(selection.color, 15s);
        if (!initialPose.has_value()) {
            publishResult(false, "selected cube pose was not received from Gazebo");
            return false;
        }

        const ArmConfiguration referencePickAbove{
            0.172849, 1.217086, -1.361123, 1.714838, 1.420795, -1.247947};
        // Tool height is about 0.277 m. This keeps the 140 mm fingers above
        // the 0.105 m table surface while overlapping the 40 mm cube.
        const ArmConfiguration referencePickGrasp{
            0.173490, 1.155967, -1.551403, 1.966232, 1.570797, -1.397307};
        const ArmConfiguration referencePickLift{
            0.172849, 1.224120, -1.299059, 1.645741, 1.390794, -1.197947};
        const ArmConfiguration referencePlaceAbove{
            1.155188, 1.125776, -1.157327, 1.602347, 1.720806, -0.215632};
        // Low release pose. The elbow-up branch keeps the arm housing clear
        // while positioning the gripper close to the destination table.
        const ArmConfiguration referencePlaceGrasp{
            1.155191, 1.058750, -1.357769, 1.869811, 1.570802, -0.415615};

        const ArmConfiguration pickAbove = rotatedForPoint(
            referencePickAbove, 0.420, -0.150,
            selection.pose.position.x, selection.pose.position.y);
        const ArmConfiguration pickGrasp = rotatedForPoint(
            referencePickGrasp, 0.420, -0.150,
            selection.pose.position.x, selection.pose.position.y);
        const ArmConfiguration pickLift = rotatedForPoint(
            referencePickLift, 0.420, -0.150,
            selection.pose.position.x, selection.pose.position.y);
        const auto destination = destinationSlots_.at(selection.color);
        const ArmConfiguration placeAbove = rotatedForPoint(
            referencePlaceAbove, 0.370, 0.296,
            destination.first, destination.second);
        const ArmConfiguration placeGrasp = rotatedForPoint(
            referencePlaceGrasp, 0.370, 0.296,
            destination.first, destination.second);
        ArmConfiguration placeWristAlign = placeAbove;
        placeWristAlign[5] = placementWristAngles_.at(selection.color);
        constexpr double openGripper = 0.025;
        constexpr double closedGripper = 0.0;
        const std::string prefix = selection.color + ":";
        const ArmConfiguration& sourceTransit =
            sourceTransitProfiles_.at(selection.color);
        const ArmConfiguration& transferTransit =
            transferTransitProfiles_.at(selection.color);

        if (!moveArm(prefix + "color_profile_source_approach", sourceTransit, 3.0) ||
            !moveArm(prefix + "move_to_pick_pregrasp", pickAbove, 3.0) ||
            !moveArm(prefix + "descend_to_cube", pickGrasp, 3.0)) {
            publishResult(false, "pickup positioning failed");
            return false;
        }
        attachCube(selection.color);
        if (!moveGripper(prefix + "close_gripper", closedGripper, 2.5)) {
            detachCube(selection.color);
            publishResult(false, "gripper close failed after attachment");
            return false;
        }
        if (!moveArm(prefix + "lift_cube", pickLift, 3.0)) {
            detachCube(selection.color);
            publishResult(false, "lift motion failed after grasp attachment");
            return false;
        }
        if (!verifyLift(selection.color, *initialPose)) {
            detachCube(selection.color);
            moveGripper(prefix + "recovery_open_gripper", openGripper, 2.0);
            moveArm(prefix + "recovery_retreat", pickAbove, 3.0);
            publishResult(false, "cube was not retained by the gripper during lift");
            return false;
        }
        if (!moveArm(prefix + "color_profile_high_transfer", transferTransit, 3.0) ||
            !moveArm(prefix + "transfer_cube", placeAbove, 4.0) ||
            !moveArm(prefix + "rotate_wrist_for_place", placeWristAlign, 1.8) ||
            !moveArm(prefix + "lower_cube", placeGrasp, 3.0)) {
            detachCube(selection.color);
            publishResult(false, "place motion or gripper controller failed");
            return false;
        }
        detachCube(selection.color);
        std::this_thread::sleep_for(500ms);
        if (!moveGripper(prefix + "open_gripper", openGripper, 2.0) ||
            !moveArm(prefix + "retreat", placeAbove, 2.5) ||
            !moveArm(prefix + "return_home", homePosition_, 4.0)) {
            publishResult(false, "release, retreat, or return-home motion failed");
            return false;
        }

        std::this_thread::sleep_for(1500ms);
        const auto finalPose = latestCubePose(selection.color);
        if (!finalPose.has_value()) {
            publishResult(false, "final selected-cube pose was unavailable");
            return false;
        }
        const double displacement = std::hypot(
            finalPose->position.x - initialPose->position.x,
            finalPose->position.y - initialPose->position.y);
        const double goalError = std::hypot(
            finalPose->position.x - destination.first,
            finalPose->position.y - destination.second);
        const double heightError = std::abs(finalPose->position.z - 0.125);
        const bool success =
            displacement > 0.20 && goalError < 0.08 && heightError < 0.08;
        const std::string detail =
            "color=" + selection.color +
            ", displacement=" + std::to_string(displacement) +
            " m, goal_error=" + std::to_string(goalError) +
            " m, final_height=" + std::to_string(finalPose->position.z) + " m";
        publishResult(success, detail);
        if (success) {
            std::lock_guard<std::mutex> lock(stateMutex_);
            completedColors_.insert(selection.color);
        }
        return success;
    }

    void runCommandLoop() {
        RCLCPP_INFO(get_logger(), "Waiting for separate arm and gripper actions");
        if (!armActionClient_->wait_for_action_server(90s) ||
            !gripperActionClient_->wait_for_action_server(90s)) {
            publishResult(false, "arm or gripper trajectory action was unavailable");
            return;
        }
        releaseAllCubes();
        if (!moveGripper("initialize_open_gripper", 0.025, 2.0) ||
            !moveArm("initialize_home", homePosition_, 4.0)) {
            publishResult(false, "could not initialize the safe home pose");
            return;
        }
        while (rclcpp::ok() && !stopRequested_.load()) {
            const auto selection = waitForSelection();
            if (!selection.has_value()) {
                return;
            }
            runSequence(*selection);
            clearSelection();
        }
    }

    const ArmConfiguration homePosition_{
        0.664000, 1.250000, -1.200000, 1.520796, 1.570796, -0.806000};

    const std::map<std::string, ArmConfiguration> sourceTransitProfiles_{
        {"white", {-0.704812, 1.750000, -0.650000, 0.470796, 1.300000, -1.100000}},
        {"red", {-0.486819, 1.650000, -0.800000, 0.720796, 1.420000, -0.950000}},
        {"blue", {-0.269525, 1.550000, -0.950000, 0.970796, 1.550000, -0.800000}},
        {"yellow", {-0.052232, 1.400000, -1.300000, 1.470796, 1.680000, -0.650000}},
        {"green", {0.165761, 1.250000, -1.550000, 1.870796, 1.780000, -0.500000}}};

    const std::map<std::string, ArmConfiguration> transferTransitProfiles_{
        {"white", {0.498115, 1.300000, -1.500000, 1.770796, 1.200000, -0.950000}},
        {"red", {0.498366, 1.400000, -1.300000, 1.470796, 1.350000, -0.750000}},
        {"blue", {0.498160, 1.550000, -0.950000, 0.970796, 1.500000, -0.600000}},
        {"yellow", {0.497954, 1.680000, -0.750000, 0.640796, 1.680000, -0.450000}},
        {"green", {0.498205, 1.780000, -0.600000, 0.390796, 1.820000, -0.300000}}};

    const std::map<std::string, double> placementWristAngles_{
        {"white", -1.100000},
        {"red", -0.800000},
        {"blue", 0.200000},
        {"yellow", 0.550000},
        {"green", 0.900000}};

    const std::vector<std::string> armJointNames_{
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    const std::vector<std::string> gripperJointNames_{
        "left_finger_joint", "right_finger_joint"};
    const std::map<std::string, std::pair<double, double>> slots_{
        {"white", {0.153, -0.419}}, {"red", {0.240, -0.376}},
        {"blue", {0.315, -0.315}}, {"yellow", {0.376, -0.240}},
        {"green", {0.419, -0.153}}};
    const std::map<std::string, std::pair<double, double>> destinationSlots_{
        {"white", {0.164, 0.449}}, {"red", {0.257, 0.403}},
        {"blue", {0.338, 0.338}}, {"yellow", {0.403, 0.257}},
        {"green", {0.449, 0.164}}};

    ActionClient::SharedPtr armActionClient_;
    ActionClient::SharedPtr gripperActionClient_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stagePublisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr resultPublisher_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr>
        attachPublishers_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr>
        detachPublishers_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr selectedPoseSubscriber_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr>
        cubePoseSubscribers_;
    mutable std::mutex stateMutex_;
    std::optional<TargetSelection> selection_;
    std::map<std::string, geometry_msgs::msg::Pose> cubePoses_;
    std::set<std::string> completedColors_;
    std::atomic<bool> stopRequested_{false};
    std::thread worker_;
};

}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboPickPlaceCoordinator>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
