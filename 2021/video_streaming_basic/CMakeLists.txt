cmake_minimum_required(VERSION 3.0.2)
project(video_streaming)

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

add_executable(rpi src/rpi.cpp)
target_link_libraries(rpi asio ${OpenCV_LIBRARIES})

add_executable(ground_station src/ground_station.cpp)
target_link_libraries(ground_station asio ${OpenCV_LIBRARIES})
