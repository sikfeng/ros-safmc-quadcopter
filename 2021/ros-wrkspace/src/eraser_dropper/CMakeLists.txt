cmake_minimum_required(VERSION 3.0.2)
project(eraser_dropper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(eraser_dropper src/eraser_dropper.cpp)
target_link_libraries(eraser_dropper ${catkin_LIBRARIES} pigpio)
