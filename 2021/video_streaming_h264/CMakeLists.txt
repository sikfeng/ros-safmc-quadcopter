cmake_minimum_required(VERSION 3.0.2)
project(video_streaming)

find_package(OpenCV REQUIRED)

find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
find_library(AVCODEC_LIBRARY avcodec)

find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
find_library(AVFORMAT_LIBRARY avformat)

find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
find_library(AVUTIL_LIBRARY avutil)

find_path(AVDEVICE_INCLUDE_DIR libavdevice/avdevice.h)
find_library(AVDEVICE_LIBRARY avdevice)

find_path(SWSCALE_INCLUDE_DIR libswscale/swscale.h)
find_library(SWSCALE_LIBRARY swscale)

add_library(libav INTERFACE)
target_include_directories(libav INTERFACE 
	${AVCODEC_INCLUDE_DIR} 
	${AVFORMAT_INCLUDE_DIR} 
	${AVUTIL_INCLUDE_DIR} 
	${AVDEVICE_INCLUDE_DIR} 
	${SWSCALE_INCLUDE_DIR}
)
target_link_libraries(libav INTERFACE 
	${AVCODEC_LIBRARY} 
	${AVFORMAT_LIBRARY} 
	${AVUTIL_LIBRARY} 
	${AVDEVICE_LIBRARY} 
	${SWSCALE_LIBRARY}
)

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
target_link_libraries(rpi asio libav ${OpenCV_LIBRARIES})

add_executable(ground_station src/ground_station.cpp)
target_link_libraries(ground_station asio libav ${OpenCV_LIBRARIES})

add_executable(benchmark src/benchmark.cpp)
target_link_libraries(benchmark asio libav ${OpenCV_LIBRARIES})
