/*****************************************************************************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

ROS bits and integration added by Florian Lier flier at techfak dot uni-bielefeld dot de

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************************************************************************/

// STD
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>
#include <unistd.h>
#include <mutex>

// ARUCO
#include "aruco.h"

// CV
#include "cvdrawingutils.h"
#include "opencv2/opencv.hpp"

// ROS
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace aruco;
using namespace cv;
using std::cout;
using std::cerr;
using std::pair;
using std::string;
using std::vector;
using std::endl;

cv::Mat current_image_copy;
cv::Mat current_image;
cv::Mat rot_mat(3, 3, cv::DataType<float>::type);

CameraParameters TheCameraParameters;
MarkerDetector MDetector;
vector<Marker> TheMarkers;

void cvTackBarEvents(int pos,void*);
bool readCameraParameters(string TheIntrinsicFile,CameraParameters &CP,Size size);

pair<double,double> AvrgTime(0,0) ;
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
bool update_images;
string TheInputVideo;
string TheIntrinsicFile;
float TheMarkerSize=-1;
std::recursive_mutex r_mutex;
const float p_off = CV_PI;
const float r_off = CV_PI/2;
const float y_off = CV_PI/2;

ros::Time timestamp;
ros::Time last_frame;


class ImageConverter
{
  Mat src_img;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  ImageConverter() : it_(nh_)
  {
    // subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/cam0/image_mono", 1, &ImageConverter::imageCb, this);
  }

  ~ImageConverter()
  {
    image_sub_.shutdown();
    printf(">> ROS Stopped Image Import \n");
  }

  void getCurrentImage(cv::Mat *input_image) {
    while((timestamp.toSec() - last_frame.toSec()) <= 0) {
        usleep(2000);
        ros::spinOnce();
    }
    r_mutex.lock();
    *input_image = src_img;
    last_frame = timestamp;
    r_mutex.unlock();
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    ros::Time frame_time = ros::Time::now();
    timestamp = frame_time;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      r_mutex.unlock();
      return;
    }
    r_mutex.lock();
    src_img = cv_ptr->image;
    r_mutex.unlock();
  }

};

bool readArguments ( int argc,char **argv )
{
    if (argc<2) {
        cerr << ">>> Invalid number of arguments" << endl;
        cerr << ">>> Usage: (in.avi|live|topic) [intrinsics.yml] [size]" <<endl;
        return false;
    }

    TheInputVideo=argv[1];

    if (argc>=3)
        TheIntrinsicFile=argv[2];
    if (argc>=4)
        TheMarkerSize=atof(argv[3]);
    if (argc==3)
        cerr<< ">>> NOTE: You need makersize to see 3d info!" <<endl;

    return true;

}

int main(int argc,char **argv) {

    // ROS messaging init
	ros::init(argc, argv, "aruco_tf_publisher");
	ros::NodeHandle n;
    ros::spinOnce();

    update_images = true;

	if (readArguments(argc,argv)==false) {
		return 0;
	}

    ImageConverter ic = ImageConverter();

    while (current_image.empty()) {
        ros::spinOnce();
        ic.getCurrentImage(&current_image);
        usleep(1000);
    }

	// Read camera parameters if passed
	if (TheIntrinsicFile != "") {
		TheCameraParameters.readFromXMLFile(TheIntrinsicFile);
		TheCameraParameters.resize(current_image.size());
	}

	// Create gui
	// cv::namedWindow("THRESHOLD IMAGE", 1);
	cv::namedWindow("ROS ARUCO", 1);

	MDetector.getThresholdParams(ThresParam1, ThresParam2);

        // MDetector.setCornerRefinementMethod(MarkerDetector::LINES);
        // 18.04 3.1.12
        // error: call to non-static member function without an object argument
        // CORNER_LINES=1
        MarkerDetector::Params::setCornerRefinementMethod(1);

        iThresParam1 = ThresParam1;
        iThresParam2 = ThresParam2;

	cv::createTrackbar("ThresParam1", "ROS ARUCO", &iThresParam1, 13, cvTackBarEvents);
	cv::createTrackbar("ThresParam2", "ROS ARUCO", &iThresParam2, 13, cvTackBarEvents);

	char key=0;

	ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("aruco/pose", 1);
    // ros::Publisher pose_pub_stamped = n.advertise<geometry_msgs::PointStamped>("aruco_point_stamped", 1);
    ros::Publisher pose_pub_markers = n.advertise<visualization_msgs::MarkerArray>("/aruco/markerarray", 1);
	tf::TransformBroadcaster broadcaster;

	// Capture until press ESC or until the end of the video
	while ((key != 'x') && (key != 27) && ros::ok()) {

   		key = cv::waitKey(1);

        ros::spinOnce();

        ic.getCurrentImage(&current_image);

        if (current_image.empty()) {
            usleep(2000);
            cout << ">>> Image EMPTY" << endl;
            continue;
        }

        // Detection of markers in the image passed
        MDetector.detect(current_image, TheMarkers, TheCameraParameters, TheMarkerSize);

        float x_t, y_t, z_t;
        float roll,yaw,pitch;

        bool found = (TheMarkers.size()>0)?true:false;

        if (found) {

            x_t = -TheMarkers[0].Tvec.at<Vec3f>(0,0)[0];
            y_t = TheMarkers[0].Tvec.at<Vec3f>(0,0)[1];
            z_t = TheMarkers[0].Tvec.at<Vec3f>(0,0)[2];

            cv::Rodrigues(TheMarkers[0].Rvec, rot_mat);

            pitch = -atan2(rot_mat.at<float>(2,0), rot_mat.at<float>(2,1));
            yaw   = acos(rot_mat.at<float>(2,2));
            roll  = -atan2(rot_mat.at<float>(0,2), rot_mat.at<float>(1,2));

        } else {
            // printf(">>> Marker _NOT_ found\n");
            usleep(2000);
            continue;
        }

        // See: http://en.wikipedia.org/wiki/Flight_dynamics
        if (found) {

            printf( "Angle >> roll: %5.1f pitch: %5.1f yaw: %5.1f \n", (roll-r_off)*(180.0/CV_PI), (pitch-p_off)*(180.0/CV_PI), (yaw-y_off)*(180.0/CV_PI));
            printf( "Dist. >>  x_d: %5.1f   y_d: %5.1f z_d: %5.1f \n", x_t, y_t, z_t);

            geometry_msgs::Pose msg;
            geometry_msgs::PointStamped msg_ps;

            if (ros::ok()) {

                // Publish TF message including the offsets
                tf::Quaternion quat = tf::createQuaternionFromRPY(roll-p_off, pitch+p_off, yaw-y_off);
                broadcaster.sendTransform(tf::StampedTransform(tf::Transform(quat, tf::Vector3(x_t, y_t, z_t)), ros::Time::now(),"camera", "marker"));

                // Now publish the pose message, remember the offsets
                msg.position.x = x_t;
                msg.position.y = y_t;
                msg.position.z = z_t;
                geometry_msgs::Quaternion p_quat = tf::createQuaternionMsgFromRollPitchYaw(roll-r_off, pitch+p_off, yaw-y_off);
                msg.orientation = p_quat;
                pose_pub.publish(msg);

                // Now publish the pose message stamped, remember the offsets
                // msg_ps.header.stamp = timestamp;
                // msg_ps.point.x = x_t;
                // msg_ps.point.y = y_t;
                // msg_ps.point.z = z_t;
                // pose_pub_stamped.publish(msg_ps);

                visualization_msgs::MarkerArray markers;
                visualization_msgs::Marker m;
                geometry_msgs::Pose pose;

                m.id = 0;
                m.header.frame_id = "aruco";
                m.header.stamp = timestamp;
                pose.position.x = x_t;
                pose.position.y = y_t;
                pose.position.z = z_t;
                pose.orientation = p_quat;
                m.pose = pose;
                markers.markers.push_back(m);
                pose_pub_markers.publish(markers);

            }
        }

        if (TheCameraParameters.isValid()){
            for (unsigned int i=0;i<TheMarkers.size();i++) {
                CvDrawingUtils::draw3dCube(current_image, TheMarkers[i], TheCameraParameters);
                CvDrawingUtils::draw3dAxis(current_image, TheMarkers[i], TheCameraParameters);
            }
        }

        // Show input with augmented information and the thresholded image
        if (update_images) {
            cv::imshow("ROS ARUCO", current_image);
            // cv::imshow("THRESHOLD IMAGE", MDetector.getThresholdedImage());
        }

        // If space is hit, don't render the image.
		if (key == ' '){
			update_images = !update_images;
		}

        // Limit to 50hz
  		usleep(20000);

	}
}

void checkbox_callback(bool value){
	update_images = value;
}

void cvTackBarEvents(int pos, void*)
{
    if (iThresParam1<3) iThresParam1=3;
    if (iThresParam1%2!=1) iThresParam1++;
    if (ThresParam2<1) ThresParam2=1;

    ThresParam1=iThresParam1;
    ThresParam2=iThresParam2;

    MDetector.setThresholdParams(ThresParam1, ThresParam2);

    // Recompute
    // MDetector.detect(current_image, TheMarkers, TheCameraParameters);
    // current_image.copyTo(current_image_copy);

    // for (unsigned int i=0;i<TheMarkers.size();i++) TheMarkers[i].draw(current_image_copy, Scalar(0,0,255), 1);

    // Draw a 3D cube in each marker if there is 3d info
    // if (TheCameraParameters.isValid())
    //     for (unsigned int i=0;i<TheMarkers.size();i++)
    //         CvDrawingUtils::draw3dCube(current_image_copy, TheMarkers[i], TheCameraParameters);

    // cv::imshow("ROS ARUCO", current_image_copy);
    // cv::imshow("THRESHOLD IMAGE", MDetector.getThresholdedImage());
}
