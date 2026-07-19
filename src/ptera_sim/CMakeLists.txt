cmake_minimum_required(VERSION 3.1)
project(ptera_sim)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

include(plugins/jointPositionController/CMakeLists.txt)
include(plugins/posePublisher/CMakeLists.txt)
#include(plugins/vehicleController/CMakeLists.txt)
include(plugins/vehicleVisualizer/CMakeLists.txt)
include(plugins/wrenchApplicator/CMakeLists.txt)

# install  files 
install(DIRECTORY 
    launch 
    models
    worlds 
    config
    DESTINATION 
    share/${PROJECT_NAME}
)

ament_package()
