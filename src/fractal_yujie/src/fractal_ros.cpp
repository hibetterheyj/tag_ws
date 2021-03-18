#include <cmath>
#include <fstream>
#include <iostream>
#include <time.h>

// ROS
#include "ros/ros.h"

// ROS sensor messages
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int16.h"

// ROS image geometry
#include <image_geometry/pinhole_camera_model.h>

// ROS transform
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <aruco_yujie/ArucoInfo.h>

// ROS CvBridge
#include "cv_bridge/cv_bridge.h"

// Image Transport to publish output img
#include <image_transport/image_transport.h>

// OpenCV
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp> // <opencv2/calib3d.hpp>

// aruco/fractal
#include <aruco.h>
#include "cvdrawingutils.h"

using namespace std;
using namespace sensor_msgs;
using namespace cv;
using namespace aruco;

// Publisher
image_transport::Publisher result_img_pub_;
ros::Publisher tf_list_pub_;
// ros::Publisher aruco_info_pub_;
ros::Publisher fractal_info_pub_;



int main(int argc, char **argv) {
    map<string, aruco::PREDEFINED_DICTIONARY_NAME> dictionary_names;
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_50", aruco::DICT_5X5_50));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_100", aruco::DICT_5X5_100));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_250", aruco::DICT_5X5_250));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_1000", aruco::DICT_5X5_1000));

    signal(SIGINT, int_handler);

    // Initalize ROS node
    ros::init(argc, argv, "aruco_ros");
    ros::NodeHandle nh("~");
    string rgb_topic, rgb_info_topic, dictionary_name;
    nh.param("camera", rgb_topic, string("/kinect2/hd/image_color_rect"));
    nh.param("camera_info", rgb_info_topic, string("/kinect2/hd/camera_info"));
    nh.param("show_detections", show_detections, true);
    nh.param("tf_prefix", marker_tf_prefix, string("marker"));
    nh.param("marker_size", marker_size, 0.09f);
    nh.param("enable_blur", enable_blur, true);
    nh.param("blur_window_size", blur_window_size, 7);
    nh.param("image_fps", image_fps, 30);
    nh.param("image_width", image_width, 640);
    nh.param("image_height", image_height, 480);
    nh.param("num_detected", num_detected, 50);
    nh.param("min_prec_value", min_prec_value, 80);

    detector_params = aruco::DetectorParameters::create();
    // FRACTAL_2L_6 FRACTAL_3L_6 FRACTAL_4L_6 FRACTAL_5L_6
    nh.param("dictionary_name", dictionary_name, string("FRACTAL_4L_6"));
    nh.param("aruco_adaptiveThreshWinSizeStep", detector_params->adaptiveThreshWinSizeStep, 4);
    int queue_size = 10;

    // Configure ARUCO marker detector
    // dictionary = aruco::getPredefinedDictionary(dictionary_names[dictionary_name]);
    ROS_DEBUG("%f", marker_size);

    if (show_detections) {
    // namedWindow("markers", cv::WINDOW_KEEPRATIO);
    }
    ros::Subscriber rgb_sub = nh.subscribe(rgb_topic.c_str(), queue_size, callback);
    ros::Subscriber rgb_info_sub = nh.subscribe(rgb_info_topic.c_str(), queue_size, callback_camera_info);
    ros::Subscriber parameter_sub = nh.subscribe("/update_params", queue_size, update_params_cb);

    // Publisher:
    image_transport::ImageTransport it(nh);
    result_img_pub_ = it.advertise("/result_img", 1);
    tf_list_pub_    = nh.advertise<tf2_msgs::TFMessage>("/tf_list", 10);

    aruco_info_pub_ = nh.advertise<aruco_yujie::ArucoInfo>("/aruco_list", 10);

    ros::spin();
    return 0;
}