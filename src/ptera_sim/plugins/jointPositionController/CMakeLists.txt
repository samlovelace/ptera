
set(ARM_PLUGIN_NAME joint_position_controller)

cmake_policy(SET CMP0057 NEW) # WONT WORK WITHOUT THIS 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(ARM_SOURCES 
  ${CMAKE_CURRENT_LIST_DIR}/JointPositionController.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../common/RateController.cpp
)

add_library(${ARM_PLUGIN_NAME} SHARED ${ARM_SOURCES})

ament_target_dependencies(${ARM_PLUGIN_NAME}
  ignition-gazebo6
  rclcpp
  std_msgs
)

include_directories(${ARM_PLUGIN_NAME} 
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/../common
)

install(TARGETS ${ARM_PLUGIN_NAME}
        LIBRARY DESTINATION lib)