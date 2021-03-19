# locate the expected locate arucoConfig.cmake
set(aruco_DIR "/home/yhe/Documents/aruco_fractal/build/")
find_package(aruco REQUIRED)

message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

include_directories(${aruco_INCLUDE_DIRS})
