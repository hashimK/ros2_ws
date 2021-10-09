cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

add_executable(px4_ros2_subscriber_example src/px4_ros2_subscriber_example.cpp)
ament_target_dependencies(px4_ros2_subscriber_example rclcpp px4_msgs)

add_executable(px4_ros2_offboard_example src/px4_ros2_offboard_example.cpp)
ament_target_dependencies(px4_ros2_offboard_example rclcpp px4_msgs)

add_executable(px4_ros2_gps_subscriber src/px4_ros2_gps_subscriber.cpp)
ament_target_dependencies(px4_ros2_gps_subscriber rclcpp px4_msgs)

add_executable(px4_ros2_read_attitude src/px4_ros2_read_attitude.cpp)
ament_target_dependencies(px4_ros2_read_attitude rclcpp px4_msgs tf2 px4_ros_com)

add_executable(px4_ros2_offboard_attitude src/px4_ros2_offboard_attitude.cpp)
ament_target_dependencies(px4_ros2_offboard_attitude rclcpp px4_msgs px4_ros_com geometry_msgs)

add_executable(px4_ros2_manual_control src/px4_ros2_manual_control.cpp)
ament_target_dependencies(px4_ros2_manual_control rclcpp px4_msgs px4_ros_com geometry_msgs)

add_executable(px4_ros2_joystick_manual_control src/px4_ros2_joystick_manual_control.cpp)
ament_target_dependencies(px4_ros2_joystick_manual_control rclcpp px4_msgs px4_ros_com geometry_msgs)

add_executable(px4_ros2_read_manual_control_switches src/px4_ros2_read_manual_control_switches.cpp)
ament_target_dependencies(px4_ros2_read_manual_control_switches rclcpp px4_msgs px4_ros_com geometry_msgs)

install(TARGETS
        cpp_node
        robot_news_station
        px4_ros2_subscriber_example
        px4_ros2_offboard_example
        px4_ros2_gps_subscriber
        px4_ros2_read_attitude
        px4_ros2_offboard_attitude
        px4_ros2_manual_control
        px4_ros2_joystick_manual_control
        px4_ros2_read_manual_control_switches
        DESTINATION lib/${PROJECT_NAME}
        )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 
