#pragma once

// #include "fractal_ros/Algorithm.hpp"
#include <csignal>
#include <iostream>
#include <map>     // used for hashmap to give certainty
#include <numeric> // used for summing a vector
#include <string>
#include <vector> // used in hashmap

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
#include "aruco_fractal/aruco_cvversioning.h"
#include "aruco_fractal/cvdrawingutils.h"
#include "aruco_fractal/fractaldetector.h"
#include <fractal_msg/FractalInfo.h>

using namespace std;
using namespace sensor_msgs;
using namespace cv;
using namespace aruco;

// 可能暂时没用
#define SSTR(x)                                                                \
  static_cast<std::ostringstream &>(std::ostringstream() << std::dec << x).str()
#define ROUND2(x) std::round(x * 100) / 100
#define ROUND3(x) std::round(x * 1000) / 1000

namespace fractal_ros {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class FractalROS {
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  FractalROS(ros::NodeHandle &nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~FractalROS();

private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /**
   * Sets up the subscribers for both camera image and info.
   */
  void setupSubscribers();

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void imageCallback(const ImageConstPtr &image_msg);
  void cameraInfoCallback(const CameraInfoConstPtr &msg);
  // void topicCallback(const sensor_msgs::Temperature &message);

  /*!
   * ROS topic callback method.
   * @param cam_info message.
   * @param useRectifiedParameters.
   * @return converted aruco::CameraParameters
   */
  aruco::CameraParameters  rosInfo2ArucoParams(const sensor_msgs::CameraInfo &cam_info,
                               bool useRectifiedParameters);

      /*!
       * ROS service server callback.
       * @param request the request of the service.
       * @param response the provided response.
       * @return true if successful, false otherwise.
       */
      // bool serviceCallback(std_srvs::Trigger::Request &request,
      //                      std_srvs::Trigger::Response &response);

      //! ROS node handle.
      ros::NodeHandle &nodeHandle_;

  //! ROS publisher
  image_transport::Publisher resultImgPublisher_;
  ros::Publisher fractalInfoPublisher_;
  ros::Publisher posePublisher_;

  //! ROS topic subscriber.
  // ros::Subscriber subscriber_;
  ros::Subscriber imageSubscriber_;
  ros::Subscriber cameraInfoSubscriber_;

  //! ROS topic name to subscribe to.
  // std::string subscriberTopic_;

  //! ROS service server.
  // ros::ServiceServer serviceServer_;

  //! Algorithm computation object.
  // Algorithm algorithm_;
  // fractal and aruco marker
  FractalDetector FDetector;
  // pose estimation
  aruco::CameraParameters cam_param;

  // Variables
  string image_topic, camera_info_topic, marker_name;
  // Use rectified parameters when reading camera info
  bool useRectifiedParameters; //  = false
  // Determine whether the camera is  read
  bool camera_model_computed; //  = false
  // Visualization
  bool show_detections, show_cube, show_axis;
  // fractal marker size
  float marker_size;
  // camera_model
  image_geometry::PinholeCameraModel camera_model;
  // image processing
  int blur_window_size; //  = 7
  bool enable_blur;     //  = true
  // camera coefficients
  Mat distortion_coefficients;
  Matx33d intrinsic_matrix;
  int image_fps; //  = 30
  int image_width; //  = 640
  int image_height; //  = 480

  //   string marker_tf_prefix;

}; // class FractalROS

} // namespace fractal_ros
