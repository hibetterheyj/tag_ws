#cmake最低版本要求
cmake_minimum_required(VERSION 2.8)
# adding 3.4.5 as main OpenCV version to be compatible with MYNT
set(OpenCV_DIR "/home/he/libs/opencv-3.4.5/build/installed/share/OpenCV")

#工程名ArUco_Test
project(sample_fractal)

#采用c++11标准编译，没有此句会报错
set(CMAKE_CXX_FLAGS "-std=c++11  -pthread")

#搜索OpenCV依赖库，cmake默认搜索/usr/local/share文件夹下的文件，下同
find_package(OpenCV REQUIRED)
# add aruco locally
find_package(Eigen3 REQUIRED)
SET(THREADLIB "pthread")

#搜索ArUco依赖库
# find_package(aruco REQUIRED)

include_directories(include/aruco_fractal/)
include_directories(include/aruco_fractal/fractallabelers/)
include_directories(
  SYSTEM
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
  # src/aruco_fractal/markerlabelers/svmmarkers.cpp
target_link_libraries(aruco_fractal
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#从main.cpp生成可执行文件ArUco_Test
add_executable(sample_detection src/sample_detection.cpp)
add_executable(sample_landing src/sample_landing.cpp)
add_executable(yujie_sample_detection src/yujie_sample_detection.cpp)
add_executable(yujie_sample_landing src/yujie_sample_landing.cpp)
add_executable(fractal_tracker src/fractal_tracker.cpp)

set(DEP_LIBS ${DEP_LIBS} ${OpenCV_LIBS})
#set(DEP_LIBS ${DEP_LIBS} ${aruco_LIBS})
set(DEP_LIBS ${DEP_LIBS} aruco_fractal)
set(DEP_LIBS ${DEP_LIBS} ${THREADLIB})

target_link_libraries(sample_detection ${DEP_LIBS})
target_link_libraries(sample_landing ${DEP_LIBS})
target_link_libraries(yujie_sample_detection ${DEP_LIBS})
target_link_libraries(yujie_sample_landing ${DEP_LIBS})
target_link_libraries(fractal_tracker ${DEP_LIBS})

# #生成文件链接至OpenCV库，注意OpenCV_LIBS变量需在OpenCVConfig.cmake文件中定义，下同
# target_link_libraries(ArUco_Test ${OpenCV_LIBS})

# #生成文件链接至aruco库
# target_link_libraries(ArUco_Test ${aruco_LIBS})