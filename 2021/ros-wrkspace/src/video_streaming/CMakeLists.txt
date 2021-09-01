cmake_minimum_required(VERSION 3.0.2)
project(video_streaming)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)

add_library(asio INTERFACE)
target_include_directories(asio
  INTERFACE ${asio_SOURCE_DIR}/asio/include
)
target_compile_definitions(asio
  INTERFACE
  ASIO_STANDALONE
  ASIO_NO_DEPRECATED
)
target_link_libraries(asio
  INTERFACE pthread
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES video_streaming
#  CATKIN_DEPENDS cv_bridge roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(video_streaming src/main.cpp)
target_link_libraries(video_streaming asio ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
