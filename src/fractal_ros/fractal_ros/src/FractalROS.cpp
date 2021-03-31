#include "fractal/FractalROS.hpp"

// STD

namespace fractal_ros {

FractalROS::FractalROS(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle) {
  /* get parameters */
  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  /* initialize ROS */
  setupSubscribers();
  ROS_INFO("Successfully launched node.");
}

FractalROS::~FractalROS()
{
}

bool FractalROS::readParameters() {
  // FractalROS parameters
  nodeHandle_.param("camera", image_topic, // rgb_topic
                    string("/mynteye/left_rect/image_rect"));
  nodeHandle_.param("camera_info", camera_info_topic, // rgb_info_topic
                    string("/mynteye/left_rect/camera_info"));
  nodeHandle_.param("show_detections", show_detections, true);
  nodeHandle_.param("show_cube", show_detections, true);
  nodeHandle_.param("show_axis", show_detections, true);
  nodeHandle_.param("tf_prefix", marker_tf_prefix, string("marker"));
  nodeHandle_.param("marker_size", marker_size, 0.2f);
  nodeHandle_.param("enable_blur", enable_blur, false);
  nodeHandle_.param("blur_window_size", blur_window_size, 7);
  nodeHandle_.param("image_fps", image_fps, 30);
  nodeHandle_.param("image_width", image_width, 752);
  nodeHandle_.param("image_height", image_height, 480);
  nodeHandle_.param("marker_name", marker_name, string("FRACTAL_5L_6"));
  int queue_size = 10;

  // if (!nodeHandle_.getParam("subscriber_topic", subscriberTopic_)) return false;
  return true;
}

void FractalROS::setupSubscribers() {
  //
  // subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
  //                                     &FractalROS::topicCallback, this);
  // serviceServer_ = nodeHandle_.advertiseService(
  //     "get_average", &FractalROS::serviceCallback, this);
  imageSubscriber_ = nodeHandle_.subscribe(image_topic.c_str(), queue_size,
                                           &FractalROS::imageCallback, this);
  cameraInfoSubscriber_ = nodeHandle_.subscribe(camera_info_topic.c_str(), queue_size,
                            &FractalROS::cameraInfoCallback, this);
}

void FractalROS::imageCallback(const ImageConstPtr &image_msg) {
  //
}

void FractalROS::cameraInfoCallback(const CameraInfoConstPtr &msg) {
  //
}

  // bool FractalROS::serviceCallback(std_srvs::Trigger::Request & request,
  //                                  std_srvs::Trigger::Response & response) {
  //   // response.success = true;
  //   // response.message = "The average is " +
  //   // std::to_string(algorithm_.getAverage()); return true;
  // }

} /* namespace */