cmake_minimum_required(VERSION 3.16)
project(cpp_robot_arm_kinematics VERSION 5.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(robot_arm
    src/robot_math.cpp
    src/robot_model.cpp
    src/inverse_kinematics.cpp
    src/trajectory_planner.cpp
    src/svg_plotter.cpp
)

target_include_directories(robot_arm PUBLIC include)

set_target_properties(robot_arm PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(MSVC)
    target_compile_options(robot_arm PRIVATE /W4 /permissive-)
else()
    target_compile_options(robot_arm PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
endif()

add_executable(robot_arm_demo src/main.cpp)
target_link_libraries(robot_arm_demo PRIVATE robot_arm)

enable_testing()
add_executable(robot_arm_tests tests/test_main.cpp)
target_link_libraries(robot_arm_tests PRIVATE robot_arm)
add_test(NAME robot_arm_tests COMMAND robot_arm_tests)

install(DIRECTORY include/ DESTINATION include)

find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
    find_package(builtin_interfaces REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(control_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(rclcpp_action REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(trajectory_msgs REQUIRED)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc)

    add_executable(color_cube_vision_node
        src/color_cube_vision_node.cpp
    )
    target_compile_options(color_cube_vision_node PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion
    )
    target_include_directories(color_cube_vision_node PRIVATE
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(color_cube_vision_node ${OpenCV_LIBS})
    ament_target_dependencies(color_cube_vision_node
        builtin_interfaces
        cv_bridge
        geometry_msgs
        rclcpp
        sensor_msgs
        std_msgs
    )

    add_executable(robot_arm_ros_node src/robot_arm_ros_node.cpp)
    target_link_libraries(robot_arm_ros_node robot_arm)
    target_compile_options(robot_arm_ros_node PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion
    )
    ament_target_dependencies(robot_arm_ros_node
        geometry_msgs
        nav_msgs
        rclcpp
        sensor_msgs
    )

    add_executable(gazebo_pick_place_coordinator
        src/gazebo_pick_place_coordinator.cpp
    )
    target_compile_options(gazebo_pick_place_coordinator PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion
    )
    ament_target_dependencies(gazebo_pick_place_coordinator
        control_msgs
        geometry_msgs
        rclcpp
        rclcpp_action
        std_msgs
        trajectory_msgs
    )

    add_executable(color_command src/color_command_cli.cpp)
    target_compile_options(color_command PRIVATE
        -Wall -Wextra -Wpedantic -Wconversion
    )
    ament_target_dependencies(color_command
        rclcpp
        std_msgs
    )

    install(TARGETS
        robot_arm
        robot_arm_demo
        robot_arm_ros_node
        color_cube_vision_node
        gazebo_pick_place_coordinator
        color_command
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
    install(DIRECTORY config launch meshes rviz urdf worlds
        DESTINATION share/${PROJECT_NAME}
    )
    install(FILES README.md PROJECT_REPORT.md CHANGELOG.md
        DESTINATION share/${PROJECT_NAME}
    )

    ament_export_include_directories(include)
    ament_export_libraries(robot_arm)
    ament_package()
else()
    install(TARGETS robot_arm robot_arm_demo)
endif()
