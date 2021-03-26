# locate the expected OpenCVConfig.cmake
set(OpenCV_DIR "/usr/share/OpenCV/")
find_package(OpenCV REQUIRED)

message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

include_directories(${OpenCV_INCLUDE_DIRS})
