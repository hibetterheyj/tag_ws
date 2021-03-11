#cmake最低版本要求
cmake_minimum_required(VERSION 2.8)

#工程名ArUco_Test
project(sample_fractal)

#采用c++11标准编译，没有此句会报错
set(CMAKE_CXX_FLAGS "-std=c++11")

#搜索OpenCV依赖库，cmake默认搜索/usr/local/share文件夹下的文件，下同
find_package(OpenCV REQUIRED)

#搜索ArUco依赖库
find_package(aruco REQUIRED)

#从main.cpp生成可执行文件ArUco_Test
add_executable(sample_detection src/sample_detection.cpp)
add_executable(sample_landing src/sample_landing.cpp)
add_executable(yujie_sample_detection src/yujie_sample_detection.cpp)
add_executable(yujie_sample_landing src/yujie_sample_landing.cpp)

set(DEP_LIBS ${DEP_LIBS} ${OpenCV_LIBS})
set(DEP_LIBS ${DEP_LIBS} ${aruco_LIBS})

target_link_libraries(sample_detection ${DEP_LIBS})
target_link_libraries(sample_landing ${DEP_LIBS})
target_link_libraries(yujie_sample_detection ${DEP_LIBS})
target_link_libraries(yujie_sample_landing ${DEP_LIBS})

# #生成文件链接至OpenCV库，注意OpenCV_LIBS变量需在OpenCVConfig.cmake文件中定义，下同
# target_link_libraries(ArUco_Test ${OpenCV_LIBS})

# #生成文件链接至aruco库
# target_link_libraries(ArUco_Test ${aruco_LIBS})