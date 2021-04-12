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
#include <geometry_msgs/PoseArray.h>
#include <marker_msgs/DegreeStamped.h>
//#include <marker_msgs/DistStamped.h>
#include <marker_msgs/PoseStamped.h>
//#include <marker_msgs/RpyDegree.h>

// ROS transform
#include <tf/tf.h>
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
#include "aruco_fractal/aruco_cvversioning.h"
#include "aruco_fractal/cvdrawingutils.h"
#include "aruco_fractal/fractaldetector.h"

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
// 0407 update distance info
// ros::Publisher MarkerDistPub;
ros::Publisher MarkerPosePub;
ros::Publisher RpyDegreePub;
// ros::Publisher MarkerPoseArrayPub;
// geometry_msgs::PoseArray pose_array;
// pose estimation
aruco::CameraParameters cam_param;
bool useRectifiedParameters = false;

// Define global variables
bool camera_model_computed = false;
bool camera_info_set = false;
bool show_detections, show_cube, show_axis;
float marker_size;
string marker_tf_prefix;
int blur_window_size = 7;
int image_fps = 30;
int image_width = 640;
int image_height = 480;
bool enable_blur;
bool fps_verbose;
int64_t ticks_image_callback, ticks_checkout_marker, ticks_compute_pose;

// hashmap used for uncertainty:
int num_detected = 10; // =0 -> not used
int min_prec_value = 80; // min precentage value to be a detected marker.

void int_handler(int x) {
  // disconnect and exit gracefully
  if (show_detections) {
    cv::destroyAllWindows();
  }
  ros::shutdown();
  exit(0);
}

aruco::CameraParameters
rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo &cam_info,
                             bool useRectifiedParameters) {
  cv::Mat cameraMatrix(3, 3, CV_64FC1);
  cv::Mat distorsionCoeff(4, 1, CV_64FC1);
  cv::Size size(cam_info.width, cam_info.height);

  if (useRectifiedParameters) {
    cameraMatrix.setTo(0);
    cameraMatrix.at<double>(0, 0) = cam_info.P[0];
    cameraMatrix.at<double>(0, 1) = cam_info.P[1];
    cameraMatrix.at<double>(0, 2) = cam_info.P[2];
    cameraMatrix.at<double>(0, 3) = cam_info.P[3];
    cameraMatrix.at<double>(1, 0) = cam_info.P[4];
    cameraMatrix.at<double>(1, 1) = cam_info.P[5];
    cameraMatrix.at<double>(1, 2) = cam_info.P[6];
    cameraMatrix.at<double>(1, 3) = cam_info.P[7];
    cameraMatrix.at<double>(2, 0) = cam_info.P[8];
    cameraMatrix.at<double>(2, 1) = cam_info.P[9];
    cameraMatrix.at<double>(2, 2) = cam_info.P[10];
    cameraMatrix.at<double>(2, 3) = cam_info.P[11];

    for (int i = 0; i < 4; ++i)
      distorsionCoeff.at<double>(i, 0) = 0;
  } else {
    for (int i = 0; i < 9; ++i)
      cameraMatrix.at<double>(i % 3, i - (i % 3) * 3) = cam_info.K[i];

    if (cam_info.D.size() == 4) {
      for (int i = 0; i < 4; ++i)
        distorsionCoeff.at<double>(i, 0) = cam_info.D[i];
    } else {
      // ROS_WARN("length of camera_info D vector is not 4, assuming zero
      // distortion...");
      ROS_WARN("length of camera_info D vector is not 4, set first 4 params");
      for (int i = 0; i < 4; ++i)
        // distorsionCoeff.at<double>(i, 0) = 0;
        distorsionCoeff.at<double>(i, 0) = cam_info.D[i];
    }
  }

  return aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
}

double _angle_to_pipi(double angle) {
  while (true) {
    if (angle < -M_PI) {
      angle += 2. * M_PI;
    }
    if (angle > M_PI) {
      angle -= 2. * M_PI;
    }
    if (abs(angle) <= M_PI) {
      break;
    }
  }
  return angle;
}

cv::Vec3d rot2euler(cv::Mat &rot) {
  double sy = sqrt(rot.at<double>(0, 0) * rot.at<double>(0, 0) +
                   rot.at<double>(1, 0) * rot.at<double>(1, 0));
  double x, y, z;
  if (sy >= 1e-6) {
    x = atan2(rot.at<double>(2, 1), rot.at<double>(2, 2));
    y = atan2(-rot.at<double>(2, 0), sy);
    z = atan2(rot.at<double>(1, 0), rot.at<double>(0, 0));
  } else {
    x = atan2(-rot.at<double>(1, 2), rot.at<double>(1, 1));
    y = atan2(-rot.at<double>(2, 0), sy);
    z = 0;
  }
  return cv::Vec3d(x, y, z);
}

void camera_info_callback(const CameraInfoConstPtr &msg) {
  if (camera_model_computed) {
    return;
  }
  // aruco_fractal
  cam_param = rosCameraInfo2ArucoCamParams(*msg, useRectifiedParameters);
  camera_model_computed = true;
  ROS_INFO("camera model is computed");
}

void image_callback(const ImageConstPtr &image_msg) {
  if (fps_verbose) {
    ticks_image_callback = cv::getTickCount();
  }
  if (!camera_model_computed) {
    ROS_INFO("camera model is not computed yet");
    return;
  }

  // set  parameters for pose estimation
  if (cam_param.isValid() && camera_info_set == false) {
    FDetector.setParams(cam_param, marker_size);
    camera_info_set = true;
    ROS_INFO("set params for fractal");
  }

  string frame_id = image_msg->header.frame_id;
  auto image = cv_bridge::toCvShare(image_msg)->image;
  cv::Mat display_image(image);

  if (fps_verbose) {
    ticks_checkout_marker = cv::getTickCount();
  }

  // Smooth the image to improve detection results
  if (enable_blur) {
    GaussianBlur(image, image, Size(blur_window_size, blur_window_size), 0, 0);
  }

  bool fractal_detected = FDetector.detect(image);

  if (fps_verbose) {
    double delta_checkout_marker =
        (double)(cv::getTickCount() - ticks_checkout_marker) /
        cv::getTickFrequency();
    cout << "checking fractal marker: " << delta_checkout_marker << " "
         << " Hz: " << 1 / delta_checkout_marker << endl;

    ticks_compute_pose = cv::getTickCount();
  }

  if (fractal_detected) {
    ROS_INFO("Detected fractal marker!");

    if (FDetector.poseEstimation()) {
      // Calc distance to marker
      cv::Mat tvec = FDetector.getTvec();
      cv::Mat rvec = FDetector.getRvec();
      cv::Mat rot;
      cv::Rodrigues(rvec, rot);
      cv::Vec3d rpy = rot2euler(rot);
      // rpy[2] = -_angle_to_pipi(rpy[2] - M_PI / 2);
      double dist =
          sqrt(pow(tvec.at<double>(0, 0), 2) + pow(tvec.at<double>(1, 0), 2) +
               pow(tvec.at<double>(2, 0), 2));
      std::cout << "Distance to fractal marker: " << dist << " meters. "
                << std::endl;
      bool publish_pose = true;
      if (publish_pose) {
        // 0407 update distance info
        // marker_msgs::DistStamped dist_stamped;
        marker_msgs::PoseStamped pose_stamped;
        marker_msgs::DegreeStamped degree_stamped;
        // dist_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.stamp = ros::Time::now();
        degree_stamped.header.stamp = pose_stamped.header.stamp;
        // dist_stamped.dist = dist;
        pose_stamped.xyz.x = tvec.at<double>(0, 0);
        pose_stamped.xyz.y = tvec.at<double>(1, 0);
        pose_stamped.xyz.z = tvec.at<double>(2, 0);
        pose_stamped.rpy.roll = rpy[0];
        pose_stamped.rpy.pitch = rpy[1];
        pose_stamped.rpy.yaw = rpy[2];
        // Roll,
        degree_stamped.rpy.roll_d = rpy[0] * 180 / M_PI;
        // Pitch, corresponds to yaw in aerial pose. left (negative) -> right (positive)
        degree_stamped.rpy.pitch_d = rpy[1] * 180 / M_PI;
        // Yaw, vertical to image axis (Z), stable is 0
        // degree_stamped.rpy.yaw_d = 90.0 - rpy[2] * 180 / M_PI;
        degree_stamped.rpy.yaw_d = rpy[2] * 180 / M_PI;
        // MarkerDistPub.publish(dist_stamped);
        MarkerPosePub.publish(pose_stamped);
        RpyDegreePub.publish(degree_stamped);
        geometry_msgs::Pose pose;
        pose.position.x = pose_stamped.xyz.x;
        pose.position.y = pose_stamped.xyz.y;
        pose.position.z = pose_stamped.xyz.z;
        pose.orientation =
            tf::createQuaternionMsgFromRollPitchYaw(rpy[0], rpy[1], rpy[2]);
        // pose_array.poses.push_back(pose);
        // pose_array.header.stamp = pose_stamped.header.stamp;
        // pose_array.header.frame_id = "fractal";
        // MarkerPoseArrayPub.publish(pose_array);
      }
    } // end of pose estimation
    if (fps_verbose) {
      double delta_compute_pose =
          (double)(cv::getTickCount() - ticks_compute_pose) /
          cv::getTickFrequency();
      cout << "computing marker pose: " << delta_compute_pose << " "
           << " Hz: " << 1 / delta_compute_pose << endl;
    }

    // Draw marker poses
    if (show_detections) {
      FDetector.drawMarkers(display_image);
      // Draw inners corners and show fractal marker
    }
    if (result_img_pub_.getNumSubscribers() > 0) {
      // yujie0325
      if (display_image.channels() == 3) {
        cv::putText(display_image, "Fractal found", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0, 255, 255), 3);
        if (FDetector.poseEstimation()) {
          FDetector.draw3d(display_image);
          FDetector.drawAxis(display_image, show_axis);
          FDetector.drawCube(display_image, show_cube);
        } else {
          FDetector.draw2d(display_image); // show at least the inner corners!
        }
        result_img_pub_.publish(
            cv_bridge::CvImage(std_msgs::Header(), "rgb8", display_image)
                .toImageMsg());
      } else if (display_image.channels() == 1) {
        cv::Mat color_display_image;
        cv::cvtColor(display_image, color_display_image, cv::COLOR_GRAY2RGB);
        cv::putText(color_display_image, "Fractal found", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0, 255, 255), 3);
        if (FDetector.poseEstimation()) {
          FDetector.draw3d(color_display_image);
          FDetector.drawAxis(color_display_image, show_axis);
          FDetector.drawCube(color_display_image, show_cube);
        } else {
          FDetector.draw2d(color_display_image);
        }
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
  if (fps_verbose) {
    double delta_image_callback =
        (double)(cv::getTickCount() - ticks_image_callback) /
        cv::getTickFrequency();
    cout << "checking image_callback: " << delta_image_callback << " "
         << " Hz: " << 1 / delta_image_callback << endl;
  }
} // end of image_callback

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
  nh.param("show_cube", show_cube, true);
  nh.param("show_axis", show_axis, true);
  nh.param("tf_prefix", marker_tf_prefix, string("marker"));
  nh.param("marker_size", marker_size, 0.2f);
  nh.param("enable_blur", enable_blur, false);
  nh.param("blur_window_size", blur_window_size, 7);
  nh.param("image_fps", image_fps, 30);
  // mynteye 752x480
  nh.param("image_width", image_width, 752);
  nh.param("image_height", image_height, 480);
  nh.param("marker_name", marker_name, string("FRACTAL_4L_6"));
  nh.param("fps_verbose", fps_verbose, true);
  int queue_size = 10;

  ros::Subscriber rgb_sub =
      nh.subscribe(rgb_topic.c_str(), queue_size, image_callback);
  ros::Subscriber c =
      nh.subscribe(rgb_info_topic.c_str(), queue_size, camera_info_callback);

  // Configure fractal marker detector
  FDetector.setConfiguration(marker_name);
  // ROS_DEBUG("%f", marker_size);
  ROS_INFO("Marker size: %f", marker_size);

  // Publisher:
  image_transport::ImageTransport it(nh);
  if (show_detections) {
    result_img_pub_ = it.advertise("/result_img", 1);
  }
  // 0407 update distance info
  MarkerPosePub = nh.advertise<marker_msgs::PoseStamped>("marker_pose", 1);
  RpyDegreePub =
      nh.advertise<marker_msgs::DegreeStamped>("marker_rpy_degree", 1);
  // MarkerPoseArrayPub =
  //     nh.advertise<geometry_msgs::PoseArray>("marker_pose_array", 1);

  ros::spin();
  return 0;
}
