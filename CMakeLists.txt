cmake_minimum_required(VERSION 3.4)
project(mobile_robot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

# all include libraries
include_directories(include)

add_definitions(-DSCRIPT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/src/plot.py")

# Robot library
add_library(lab2 SHARED src/robot.cpp include/robot.h include/geom.h
      include/envir.h src/envir.cpp
      include/sensor.h include/rangesensor.h
      include/bearingsensor.h)

# Lab 3: using the robot class
# Lab 4: sensors on robot
#add_executable( rangesensor.h)
#target_link_libraries(mobile_robot lab2)

add_executable(move_robot src/main.cpp)
target_link_libraries(move_robot lab2)

