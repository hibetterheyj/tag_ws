cmake_minimum_required(VERSION 2.8)
project(fractal_ros)
# adding 3.4.5 as main OpenCV version to be compatible with MYNT
set(OpenCV_DIR "/home/he/libs/opencv-3.4.5/build/installed/share/OpenCV")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        image_geometry
        geometry_msgs
        roscpp
        rospy
        std_msgs
        tf2
        tf2_ros
        image_transport
        aruco_fractal
        fractal_msg
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS
        cv_bridge
        image_geometry
        geometry_msgs
        std_msgs roscpp
        rospy
        tf2
        tf2_ros
        image_transport
        aruco_fractal
        fractal_msg
)

###########
## Build ##
###########

include_directories(
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/fractal_ros.cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
         ${OpenCV_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#        launch/detector.launch
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        )

