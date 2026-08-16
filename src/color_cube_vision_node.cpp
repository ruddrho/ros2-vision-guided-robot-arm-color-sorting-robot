#include <cv_bridge/cv_bridge.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {

struct Detection {
    cv::Point2d centroid;
    cv::Rect bounds;
    double area{0.0};
};

struct WorkspaceSlot {
    double x;
    double y;
};

class ColorCubeVisionNode : public rclcpp::Node {
public:
    ColorCubeVisionNode()
        : Node("color_cube_vision") {
        minimumArea_ = declare_parameter<double>("minimum_blob_area", 120.0);
        maximumArea_ = declare_parameter<double>("maximum_blob_area", 2500.0);
        commandSubscriber_ = create_subscription<std_msgs::msg::String>(
            "/target_color", rclcpp::QoS(10).reliable(),
            [this](const std_msgs::msg::String::SharedPtr message) {
                handleCommand(message->data);
            });
        imageSubscriber_ = create_subscription<sensor_msgs::msg::Image>(
            "/overhead_camera/image", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {
                processImage(message);
            });
        selectedPosePublisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/vision/selected_cube_pose",
            rclcpp::QoS(1).transient_local().reliable());
        statusPublisher_ = create_publisher<std_msgs::msg::String>(
            "/vision/status", rclcpp::QoS(1).transient_local().reliable());
        debugImagePublisher_ = create_publisher<sensor_msgs::msg::Image>(
            "/vision/debug_image", rclcpp::QoS(2).reliable());
        publishStatus("ready: command white, red, blue, yellow, or green");
    }

private:
    static std::string normalized(std::string value) {
        value.erase(
            std::remove_if(value.begin(), value.end(), [](unsigned char character) {
                return std::isspace(character) != 0;
            }),
            value.end());
        std::transform(
            value.begin(), value.end(), value.begin(), [](unsigned char character) {
                return static_cast<char>(std::tolower(character));
            });
        return value;
    }

    bool supported(const std::string& color) const {
        return slots_.find(color) != slots_.end();
    }

    void handleCommand(const std::string& rawCommand) {
        const std::string color = normalized(rawCommand);
        if (!supported(color)) {
            publishStatus("rejected unsupported color: " + color);
            RCLCPP_ERROR(
                get_logger(),
                "Unsupported color '%s'. Use white, red, blue, yellow, or green.",
                color.c_str());
            return;
        }
        requestedColor_ = color;
        commandPending_ = true;
        publishStatus("searching for " + color + " cube");
        RCLCPP_INFO(get_logger(), "Received target-color command: %s", color.c_str());
    }

    static cv::Mat thresholdForColor(const cv::Mat& hsv, const std::string& color) {
        cv::Mat mask;
        if (color == "white") {
            cv::inRange(hsv, cv::Scalar(0, 0, 185), cv::Scalar(179, 55, 255), mask);
        } else if (color == "red") {
            cv::Mat lowRed;
            cv::Mat highRed;
            cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), lowRed);
            cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(179, 255, 255), highRed);
            cv::bitwise_or(lowRed, highRed, mask);
        } else if (color == "blue") {
            cv::inRange(hsv, cv::Scalar(95, 100, 55), cv::Scalar(135, 255, 255), mask);
        } else if (color == "yellow") {
            cv::inRange(hsv, cv::Scalar(18, 110, 90), cv::Scalar(38, 255, 255), mask);
        } else {
            cv::inRange(hsv, cv::Scalar(40, 80, 45), cv::Scalar(90, 255, 255), mask);
        }
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        return mask;
    }

    std::optional<Detection> largestDetection(
        const cv::Mat& hsv, const std::string& color) const {
        cv::Mat mask = thresholdForColor(hsv, color);
        const int sourceRegionStart = static_cast<int>(
            std::lround(static_cast<double>(mask.cols) * 0.55));
        mask(cv::Rect(0, 0, sourceRegionStart, mask.rows)).setTo(0);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::optional<Detection> best;
        for (const auto& contour : contours) {
            const double area = cv::contourArea(contour);
            if (area < minimumArea_ || area > maximumArea_ ||
                (best.has_value() && area <= best->area)) {
                continue;
            }
            const cv::Rect bounds = cv::boundingRect(contour);
            const double aspectRatio = static_cast<double>(bounds.width) /
                static_cast<double>(std::max(bounds.height, 1));
            if (aspectRatio < 0.55 || aspectRatio > 1.80) {
                continue;
            }
            const cv::Moments moments = cv::moments(contour);
            if (std::abs(moments.m00) < 1.0e-9) {
                continue;
            }
            best = Detection{
                cv::Point2d(moments.m10 / moments.m00, moments.m01 / moments.m00),
                bounds, area};
        }
        return best;
    }

    void processImage(const sensor_msgs::msg::Image::ConstSharedPtr& message) {
        cv_bridge::CvImagePtr converted;
        try {
            converted = cv_bridge::toCvCopy(
                message, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& error) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 5000, "cv_bridge error: %s", error.what());
            return;
        }

        cv::Mat hsv;
        cv::cvtColor(converted->image, hsv, cv::COLOR_BGR2HSV);
        cv::Mat annotated = converted->image.clone();
        std::map<std::string, Detection> detections;
        for (const std::string& color : colors_) {
            const auto detection = largestDetection(hsv, color);
            if (!detection.has_value()) {
                continue;
            }
            detections[color] = *detection;
            cv::rectangle(annotated, detection->bounds, drawingColors_.at(color), 2);
            cv::circle(annotated, detection->centroid, 4, drawingColors_.at(color), -1);
            cv::putText(
                annotated, color, detection->bounds.tl() + cv::Point(0, -6),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, drawingColors_.at(color), 2);
        }

        if (commandPending_) {
            const auto found = detections.find(requestedColor_);
            if (found != detections.end()) {
                publishSelection(requestedColor_, found->second, message->header.stamp);
                commandPending_ = false;
            }
        }

        auto debugMessage = cv_bridge::CvImage(
            message->header, sensor_msgs::image_encodings::BGR8, annotated).toImageMsg();
        debugImagePublisher_->publish(*debugMessage);
    }

    void publishSelection(
        const std::string& color,
        const Detection& detection,
        const builtin_interfaces::msg::Time& stamp) {
        const WorkspaceSlot slot = slots_.at(color);
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = "base_link";
        pose.pose.position.x = slot.x;
        pose.pose.position.y = slot.y;
        pose.pose.position.z = 0.125;
        pose.pose.orientation.w = 1.0;
        selectedPosePublisher_->publish(pose);

        publishStatus(
            "selected " + color + " cube at pixel (" +
            std::to_string(static_cast<int>(std::lround(detection.centroid.x))) + ", " +
            std::to_string(static_cast<int>(std::lround(detection.centroid.y))) +
            "), area=" + std::to_string(static_cast<int>(std::lround(detection.area))));
        RCLCPP_INFO(
            get_logger(),
            "OpenCV selected %s cube: pixel=(%.1f, %.1f), area=%.1f, workspace=(%.3f, %.3f)",
            color.c_str(), detection.centroid.x, detection.centroid.y, detection.area,
            slot.x, slot.y);
    }

    void publishStatus(const std::string& value) {
        std_msgs::msg::String message;
        message.data = value;
        statusPublisher_->publish(message);
    }

    const std::array<std::string, 5> colors_{"white", "red", "blue", "yellow", "green"};
    const std::map<std::string, WorkspaceSlot> slots_{
        {"white", {0.153, -0.419}}, {"red", {0.240, -0.376}},
        {"blue", {0.315, -0.315}}, {"yellow", {0.376, -0.240}},
        {"green", {0.419, -0.153}}};
    const std::map<std::string, cv::Scalar> drawingColors_{
        {"white", cv::Scalar(245, 245, 245)}, {"red", cv::Scalar(30, 30, 230)},
        {"blue", cv::Scalar(220, 80, 20)}, {"yellow", cv::Scalar(20, 220, 240)},
        {"green", cv::Scalar(40, 190, 40)}};

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr commandSubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr selectedPosePublisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImagePublisher_;
    std::string requestedColor_;
    bool commandPending_{false};
    double minimumArea_{120.0};
    double maximumArea_{2500.0};
};

}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorCubeVisionNode>());
    rclcpp::shutdown();
    return 0;
}
