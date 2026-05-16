cmake_minimum_required(VERSION 3.8)
project(ptera_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommandAck.msg"
  "msg/ControllerStatus.msg"
  "msg/Enable.msg"
  "msg/Euler.msg"
  "msg/FoundObjectResponse.msg"
  "msg/GpcGoal.msg"
  "msg/JointPositionWaypoint.msg"
  "msg/JointVelocityWaypoint.msg"
  "msg/ManipulationCommand.msg"
  "msg/ManipulatorState.msg"
  "msg/PlanCommand.msg"
  "msg/PlanResponse.msg"
  "msg/Quaternion.msg"
  "msg/RobotState.msg"
  "msg/TaskPositionWaypoint.msg"
  "msg/TaskVelocityWaypoint.msg"
  "msg/Vec3.msg"
  "msg/VehicleWaypoint.msg"
  "msg/VisionCommand.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_generators std_msgs geometry_msgs)
ament_package()
