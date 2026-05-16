
set(PLUGIN_NAME vehicle_visualizer)

cmake_policy(SET CMP0057 NEW) # WONT WORK WITHOUT THIS 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ptera_msgs)

set(SOURCES 
  ${CMAKE_CURRENT_LIST_DIR}/VehicleVisualizer.cpp
  ${CMAKE_CURRENT_LIST_DIR}/../common/RateController.cpp
)

add_library(${PLUGIN_NAME} SHARED ${SOURCES})

ament_target_dependencies(${PLUGIN_NAME}
  ignition-gazebo6
  rclcpp
  ptera_msgs
)

include_directories(${PLUGIN_NAME} 
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/../common
)

install(TARGETS ${PLUGIN_NAME}
        LIBRARY DESTINATION lib)