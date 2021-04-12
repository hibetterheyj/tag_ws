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

  // Configure fractal marker detector
  FractalDetector FDetector;
  FDetector.setConfiguration(marker_name);
  ROS_INFO("Marker size: %f", marker_size);

  // Set camera parms for pose estimation
  if (cam_param.isValid()) {
    FDetector.setParams(cam_param, marker_size);
    // ROS_INFO("Set cam_param !");
  }

  // Publisher:
  image_transport::ImageTransport it;
  result_img_pub_ = it.advertise("/result_img", 1);
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
  nodeHandle_.param("show_cube", show_cube, true);
  nodeHandle_.param("show_axis", show_axis, true);
  nodeHandle_.param("tf_prefix", marker_tf_prefix, string("marker"));
  nodeHandle_.param("marker_size", marker_size, 0.2f);
  nodeHandle_.param("enable_blur", enable_blur, false);
  nodeHandle_.param("blur_window_size", blur_window_size, 7);
  nodeHandle_.param("image_fps", image_fps, 30);
  nodeHandle_.param("image_width", image_width, 752);
  nodeHandle_.param("image_height", image_height, 480);
  nodeHandle_.param("marker_name", marker_name, string("FRACTAL_5L_6"));
  nodeHandle_.param("verbose", verbose, true);
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

// aruco::CameraParameters
auto FractalROS::rosInfo2ArucoParams(const sensor_msgs::CameraInfo &cam_info,
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

    for (int i = 0; i < 4; ++i) {distorsionCoeff.at<double>(i, 0) = 0;}
  } else {
    for (int i = 0; i < 9; ++i)
      cameraMatrix.at<double>(i % 3, i - (i % 3) * 3) = cam_info.K[i];

    if (cam_info.D.size() == 4) {
      for (int i = 0; i < 4; ++i)
        distorsionCoeff.at<double>(i, 0) = cam_info.D[i];
    } else {
      // ROS_WARN("length of camera_info D vector is not 4, assuming zero distortion...");
      ROS_WARN("length of camera_info D vector is not 4, set first 4 params");
      for (int i = 0; i < 4; ++i)
        // distorsionCoeff.at<double>(i, 0) = 0;
        distorsionCoeff.at<double>(i, 0) = cam_info.D[i];
    }
  }

  return aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
}

void FractalROS::imageCallback(const ImageConstPtr &image_msg) {
  if (!camera_model_computed) {
    ROS_INFO("camera model is not computed yet");
    return;
  }

  // pose estimation
  if (cam_param.isValid()) {
    FDetector.setParams(cam_param, marker_size);
    // ROS_INFO("Set cam_param !");
  }

  string frame_id = image_msg->header.frame_id;
  auto image = cv_bridge::toCvShare(image_msg)->image;
  cv::Mat display_image(image);

  if (verbose){
    int64_t ticks = cv::getTickCount();
  }

  // Smooth the image to improve detection results
  if (enable_blur) {
    GaussianBlur(image, image, Size(blur_window_size, blur_window_size), 0, 0);
  }

  bool fractal_detected = FDetector.detect(image);
  if (verbose){
    double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
    cout << "checking fractal marker: " << delta << " fps: " << 1 / delta << endl;
  }

  if (fractal_detected) {
    ROS_INFO("Detected fractal marker!");

    if (FDetector.poseEstimation()) {
      // Calc distance to marker
      cv::Mat tvec = FDetector.getTvec();
      double Z =
          sqrt(pow(tvec.at<double>(0, 0), 2) + pow(tvec.at<double>(1, 0), 2) +
               pow(tvec.at<double>(2, 0), 2));
      if (verbose) {
        std::cout << "Distance to fractal marker: " << Z << " meters. "
                  << std::endl;
      }
    }

    // Draw marker poses
    if (show_detections) {
      // aruco::drawDetectedMarkers(display_image, corners, ids);
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
          // FDetector.drawAxis(display_image, show_axis);
          // FDetector.drawCube(display_image, show_cube);
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
          // FDetector.drawAxis(display_image, show_axis);
          // FDetector.drawCube(display_image, show_cube);
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
  } // if no markers are detected
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