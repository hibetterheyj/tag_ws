#include <csignal>
#include <iostream>
#include <map>     // used for hashmap to give certainty
#include <numeric> // used for summing a vector
#include <vector>  // used in hashmap

// ROS
#include "ros/ros.h"

// ROS sensor messages
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int16.h"

// ROS image geometry
#include <image_geometry/pinhole_camera_model.h>

// ROS transform
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>

// ROS CvBridge
#include "cv_bridge/cv_bridge.h"

// Image Transport to publish output img
#include <image_transport/image_transport.h>

// OpenCV
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// aruco_fractal
#include "aruco_fractal/aruco.h"
#include "aruco_fractal/cvdrawingutils.h"
#include <fractal_msg/FractalInfo.h>

using namespace std;
using namespace sensor_msgs;
using namespace cv;
using namespace aruco;

#define SSTR(x)                                                                \
  static_cast<std::ostringstream &>(std::ostringstream() << std::dec << x).str()
#define ROUND2(x) std::round(x * 100) / 100
#define ROUND3(x) std::round(x * 1000) / 1000

// Publisher
image_transport::Publisher result_img_pub_;
ros::Publisher fractal_info_pub_;
FractalDetector FDetector;

// Define global variables
bool camera_model_computed = false;
bool show_detections;
float marker_size;
image_geometry::PinholeCameraModel camera_model;
Mat distortion_coefficients;
Matx33d intrinsic_matrix;
string marker_tf_prefix;
int blur_window_size = 7;
int image_fps = 30;
int image_width = 640;
int image_height = 480;
bool enable_blur = true;

// hashmap used for uncertainty:
int num_detected = 10;   // =0 -> not used
int min_prec_value = 80; // min precentage value to be a detected marker.
map<int, std::vector<int>>
    ids_hashmap; // key: ids, value: number within last 100 imgs

void int_handler(int x) {
  // disconnect and exit gracefully
  if (show_detections) {
    cv::destroyAllWindows();
  }
  ros::shutdown();
  exit(0);
}

tf2::Vector3 cv_vector3d_to_tf_vector3(const Vec3d &vec) {
  return {vec[0], vec[1], vec[2]};
}

tf2::Quaternion cv_vector3d_to_tf_quaternion(const Vec3d &rotation_vector) {
  Mat rotation_matrix;
  auto ax = rotation_vector[0], ay = rotation_vector[1],
       az = rotation_vector[2];
  auto angle = sqrt(ax * ax + ay * ay + az * az);
  auto cosa = cos(angle * 0.5);
  auto sina = sin(angle * 0.5);
  auto qx = ax * sina / angle;
  auto qy = ay * sina / angle;
  auto qz = az * sina / angle;
  auto qw = cosa;
  tf2::Quaternion q;
  q.setValue(qx, qy, qz, qw);
  return q;
}

tf2::Transform create_transform(const Vec3d &tvec,
                                const Vec3d &rotation_vector) {
  tf2::Transform transform;
  transform.setOrigin(cv_vector3d_to_tf_vector3(tvec));
  transform.setRotation(cv_vector3d_to_tf_quaternion(rotation_vector));
  return transform;
}

void callback_camera_info(const CameraInfoConstPtr &msg) {
  if (camera_model_computed) {
    return;
  }
  camera_model.fromCameraInfo(msg);
  camera_model.distortionCoeffs().copyTo(distortion_coefficients);
  intrinsic_matrix = camera_model.intrinsicMatrix();
  camera_model_computed = true;
  ROS_INFO("camera model is computed");
}

void callback(const ImageConstPtr &image_msg) {
  if (!camera_model_computed) {
    ROS_INFO("camera model is not computed yet");
    return;
  }

  string frame_id = image_msg->header.frame_id;
  auto image = cv_bridge::toCvShare(image_msg)->image;
  cv::Mat display_image(image);

  int64_t ticks = cv::getTickCount();

  // Smooth the image to improve detection results
  if (enable_blur) {
    GaussianBlur(image, image, Size(blur_window_size, blur_window_size), 0, 0);
  }

  // yujie0325
  bool fractal_detected = FDetector.detect(image);

  double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
  cout << "checking aruco: " << delta << " "
       << " fps: " << 1 / delta << endl;

  // fractal_detected
  // TODO: compute pose!
  // not implemented in original exmaple code
  if (fractal_detected) {
    ROS_INFO("Detected fractal marker!");

    // Draw marker poses
    if (show_detections) {
      // aruco::drawDetectedMarkers(display_image, corners, ids);
      FDetector.drawMarkers(display_image);
      // Draw inners corners and show fractal marker
      FDetector.draw2d(display_image);
    }
    if (result_img_pub_.getNumSubscribers() > 0) {
      // yujie0325
      if (display_image.channels() == 3) {
        cv::putText(display_image, "Fractal found", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0, 255, 255), 3);
        result_img_pub_.publish(
            cv_bridge::CvImage(std_msgs::Header(), "rgb8", display_image)
                .toImageMsg());
      } else if (display_image.channels() == 1) {
        cv::Mat color_display_image;
        cv::cvtColor(display_image, color_display_image, cv::COLOR_GRAY2RGB);
        cv::putText(color_display_image, "Fractal found", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0, 255, 255), 3);
        result_img_pub_.publish(
            cv_bridge::CvImage(std_msgs::Header(), "rgb8", color_display_image)
                .toImageMsg());
      } else {
        ROS_ERROR("Unsupported channel number");
      }
    }
    auto key = waitKey(1);
    if (key == 27) {
      ROS_INFO("ESC pressed, exit the program");
      ros::shutdown();
    }

  } //  end if fractal_detected
  else {
    // if no markers are detected
    ROS_INFO("Markers not found");
    if (show_detections) {
      if (result_img_pub_.getNumSubscribers() > 0) {
        if (display_image.channels() == 3) {
          cv::putText(display_image, "Not found", cv::Point(10, 30),
                      cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 3);
          result_img_pub_.publish(
              cv_bridge::CvImage(std_msgs::Header(), "rgb8", display_image)
                  .toImageMsg());
        } else if (display_image.channels() == 1) {
          cv::Mat color_display_image;
          cv::cvtColor(display_image, color_display_image, cv::COLOR_GRAY2RGB);
          cv::putText(color_display_image, "Not found", cv::Point(10, 30),
                      cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 3);
          result_img_pub_.publish(
              // https://docs.ros.org/en/diamondback/api/cv_bridge/html/c++/cv__bridge_8cpp_source.html
              cv_bridge::CvImage(std_msgs::Header(), "rgb8",
                                 color_display_image)
                  .toImageMsg());
        } else {
          ROS_ERROR("Unsupported channel number");
        }
      } // end of result_img_pub_.getNumSubscribers
      auto key = waitKey(1);
      if (key == 27) {
        ROS_INFO("ESC pressed, exit the program");
        ros::shutdown();
      }
    }
  }

} // end of callback

int main(int argc, char **argv) {

  signal(SIGINT, int_handler);

  // Initalize ROS node
  ros::init(argc, argv, "fractal_ros");
  ros::NodeHandle nh("~");
  // string rgb_topic, rgb_info_topic, dictionary_name;
  string rgb_topic, rgb_info_topic, marker_name;
  nh.param("camera", rgb_topic, string("/mynteye/left_rect/image_rect"));
  nh.param("camera_info", rgb_info_topic,
           string("/mynteye/left_rect/camera_info"));
  nh.param("show_detections", show_detections, true);
  nh.param("tf_prefix", marker_tf_prefix, string("marker"));
  nh.param("marker_size", marker_size, 0.2f);
  nh.param("enable_blur", enable_blur, false);
  nh.param("blur_window_size", blur_window_size, 7);
  nh.param("image_fps", image_fps, 30);
  // mynteye 752x480
  nh.param("image_width", image_width, 752);
  nh.param("image_height", image_height, 480);
  nh.param("num_detected", num_detected, 1);
  nh.param("min_prec_value", min_prec_value, 80);
  nh.param("marker_name", marker_name, string("FRACTAL_5L_6"));
  int queue_size = 10;

  // Configure fractal marker detector
  FDetector.setConfiguration(marker_name);
  ROS_DEBUG("%f", marker_size);

  if (show_detections) {
  }
  ros::Subscriber rgb_sub =
      nh.subscribe(rgb_topic.c_str(), queue_size, callback);
  ros::Subscriber c =
      nh.subscribe(rgb_info_topic.c_str(), queue_size, callback_camera_info);

  // Publisher:
  image_transport::ImageTransport it(nh);
  result_img_pub_ = it.advertise("/result_img", 1);

  ros::spin();
  return 0;
}
