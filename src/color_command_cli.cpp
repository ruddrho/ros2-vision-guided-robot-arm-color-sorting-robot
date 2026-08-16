#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace {

using namespace std::chrono_literals;

std::string normalized(std::string value) {
    std::transform(
        value.begin(), value.end(), value.begin(), [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return value;
}

bool supported(const std::string& color) {
    constexpr std::array<const char*, 5> colors{
        "white", "red", "blue", "yellow", "green"};
    return std::any_of(colors.begin(), colors.end(), [&color](const char* candidate) {
        return color == candidate;
    });
}

}  // namespace

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    if (argc != 2) {
        std::cerr << "Usage: ros2 run cpp_robot_arm_kinematics color_command "
                  << "<white|red|blue|yellow|green>\n";
        rclcpp::shutdown();
        return 2;
    }

    const std::string color = normalized(argv[1]);
    if (!supported(color)) {
        std::cerr << "Unsupported color '" << color
                  << "'. Use white, red, blue, yellow, or green.\n";
        rclcpp::shutdown();
        return 2;
    }

    auto node = std::make_shared<rclcpp::Node>("color_command_cli");
    auto publisher = node->create_publisher<std_msgs::msg::String>(
        "/target_color", rclcpp::QoS(10).reliable());

    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (rclcpp::ok() && publisher->get_subscription_count() == 0U &&
           std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(100ms);
    }
    if (publisher->get_subscription_count() == 0U) {
        std::cerr << "No /target_color subscriber found. Start the Gazebo launch first.\n";
        rclcpp::shutdown();
        return 1;
    }

    std_msgs::msg::String message;
    message.data = color;
    for (int count = 0; count < 3; ++count) {
        publisher->publish(message);
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(250ms);
    }
    std::cout << "Sent color command: " << color << '\n';
    rclcpp::shutdown();
    return 0;
}
