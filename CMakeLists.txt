cmake_minimum_required(VERSION 3.16)
project(waiko_ros2_control LANGUAGES C CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(state-observation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/WaikoContact.msg
  msg/WaikoContactState.msg
  msg/WaikoInitialState.msg
  msg/WaikoInput.msg
  msg/WaikoState.msg
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs
)
rosidl_get_typesupport_target(waiko_msgs_target ${PROJECT_NAME} rosidl_typesupport_cpp)

generate_parameter_library(waiko_estimator_parameters src/waiko_estimator_parameters.yaml)
generate_parameter_library(waiko_controller_parameters src/waiko_controller_parameters.yaml)

# WAIKO with the leg odometry, and its conversions from and to ROS parameters and messages.
add_library(waiko_estimator SHARED
  src/waiko_estimator.cpp
  src/waiko_conversions.cpp
)
target_include_directories(waiko_estimator PUBLIC include)
target_link_libraries(waiko_estimator PUBLIC
  state-observation::state-observation
  tf2_eigen::tf2_eigen
  waiko_estimator_parameters
  ${waiko_msgs_target}
)

# ros2_control controller plugin.
add_library(waiko_controller SHARED
  src/waiko_controller.cpp
)
target_link_libraries(waiko_controller PUBLIC
  controller_interface::controller_interface
  pluginlib::pluginlib
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${nav_msgs_TARGETS}
  waiko_controller_parameters
  waiko_estimator
)
pluginlib_export_plugin_description_file(controller_interface waiko_ros2_control.xml)

# Estimator fed by topics, and the replay of its inputs from a bag.
add_executable(waiko_estimator_node src/waiko_estimator_node.cpp)
target_link_libraries(waiko_estimator_node PRIVATE rclcpp::rclcpp waiko_estimator)

add_executable(rosbag_publish_waiko src/rosbag_publish_waiko.cpp)
target_link_libraries(rosbag_publish_waiko PRIVATE
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
  ${waiko_msgs_target}
)

install(TARGETS waiko_estimator waiko_controller
  LIBRARY DESTINATION lib
)
install(TARGETS waiko_estimator_node rosbag_publish_waiko
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_waiko_estimator test/test_waiko_estimator.cpp)
  target_link_libraries(test_waiko_estimator waiko_estimator)

  ament_add_gmock(test_waiko_controller test/test_waiko_controller.cpp)
  target_compile_definitions(test_waiko_controller PRIVATE
    TEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
  target_link_libraries(test_waiko_controller
    waiko_controller
    controller_manager::controller_manager
    hardware_interface::hardware_interface
    ros2_control_test_assets::ros2_control_test_assets
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
