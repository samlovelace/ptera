cmake_minimum_required(VERSION 3.8)
project(ptera_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ptera_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
set(PCL_DIR "/usr/local/pcl-1.15/lib/cmake/pcl")
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

set(SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/CommanderComms.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/InputHandler.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/PointCloudHandler.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

include_directories(${PROJECT_NAME} 
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ptera_msgs
  pcl_msgs
  pcl_conversions
  Eigen3
)

install(TARGETS ${PROJECT_NAME} 
	DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY objects/ DESTINATION share/${PROJECT_NAME}/objects)
ament_package()
