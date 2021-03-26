cmake_minimum_required(VERSION 2.8)
project(aruco_fractal)
# adding 3.4.5 as main OpenCV version to be compatible with MYNT
#set(OpenCV_DIR "/home/he/libs/opencv-3.4.5/build/installed/share/OpenCV")
set(OpenCV_DIR "/home/lis/opencv-3.4.5/build/installed/share/OpenCV")

find_package(catkin)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
set(CMAKE_CXX_FLAGS "-std=c++11  -pthread")
# set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco_fractal
)
include_directories(include/aruco_fractal/)
include_directories(include/aruco_fractal/fractallabelers/)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(aruco_fractal
  src/aruco_fractal/cvdrawingutils.cpp
  src/aruco_fractal/cameraparameters.cpp
  src/aruco_fractal/debug.cpp
  src/aruco_fractal/dictionary.cpp
  src/aruco_fractal/dictionary_based.cpp
  src/aruco_fractal/ippe.cpp
  src/aruco_fractal/marker.cpp
  src/aruco_fractal/markerdetector_impl.cpp
  src/aruco_fractal/markerdetector.cpp
  src/aruco_fractal/markerlabeler.cpp
  src/aruco_fractal/markermap.cpp
  src/aruco_fractal/posetracker.cpp
  src/aruco_fractal/dictionary_based.cpp
  src/aruco_fractal/fractallabelers/fractalposetracker.cpp
  src/aruco_fractal/fractallabelers/fractalmarkerset.cpp
  src/aruco_fractal/fractallabelers/fractalmarker.cpp
  src/aruco_fractal/fractallabelers/fractallabeler.cpp
  src/aruco_fractal/fractaldetector.cpp
)
target_link_libraries(aruco_fractal
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


#############
## Install ##
#############

install(TARGETS aruco_fractal
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

