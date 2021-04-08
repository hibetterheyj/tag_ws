# Marker detection

⭐ some huge files can be found [here](https://drive.google.com/drive/folders/1qwV2kRVkfSNCs9ddREWbypiysS995Kv9?usp=sharing)

## TODO

- [x] preliminary whycon implementation(21.3.10)
- [x] preliminary aruco implementation (21.3.10)
- [x preliminary fractal implementation with ROS implementation (21.3.25)
  - [x] build a minimal library for aruco and fractal
- [x] time analysis
  - [x] preliminary test (21.3.11)
  - [ ] using gprof mentioned in [Zhihu](https://www.zhihu.com/question/265131281)
  - [ ] update all time performance on JestonNano (21.3.26)
- [ ] add badge using https://shields.io/
  clang, ros-melodic, opencv-3.2/3.4, aruco-3.1.12
  3.4.5
- [ ] add resources
  - [x] calibration and tag files (21.3.13)
  - [x] update preliminary rosbags (21.3.11)
  - [ ] update videos and camera parameter files in drone scenarios
- [ ] add CI/CD
- [ ] update OpenCV3.4.5 compilation procedure
- [x] update MyntEye SDK repo for jestonNano: https://github.com/hibetterheyj/MYNT-EYE-S-SDK/tree/jeston (21.3.25)

## Work with camera

### guvcview

- sudo apt install guvcview
    1. Check the available resolutions, framerates and formats of your USB camera by running: `guvcview --device=/dev/video1`

    2. Run guvcview and adjust your camera settings (exposure, brightness etc)

### shell

- list cameras and its permissions: `ll /dev/video*`
- cannot access camera: `chmod 777 /dev/video1`

---

## ROS package

### cv_camera

- launch files
  - calibrate

    ```shell
    roslaunch cv_camera cv_camera_only.launch use_rect:=False
    # START A NEW TEMINNAL
    rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.020 image:=/cv_camera/image_raw camera:=/cv_camera
    ```

  - view only

    ```shell
    roslaunch cv_camera cv_camera_view.launch
    roslaunch cv_camera cv_camera_view.launch device_id:=1
    ```

  - rectified images
    - method1 (two files)

      ```shell
      roslaunch cv_camera cv_camera_only.launch
      roslaunch cv_camera rect_view.launch
      ```

    - method2 (one file)

      ```shell
      roslaunch cv_camera cv_camera_view_rect.launch
      ```

### [lrse/whycon](https://github.com/lrse/whycon)

- [wiki](https://github.com/lrse/whycon/wiki)
  - [Reference: A description of the ROS nodes can be found here](https://github.com/lrse/whycon/wiki/Reference)
  - [Usage Tutorial: See a step-by-step tutorial here.](https://github.com/lrse/whycon/wiki/Tutorial)

- **run the code**

  ```shell
  # change device depends on connection
  roslaunch cv_camera cv_camera_only.launch device_id:=1
  # with RVIZ
  roslaunch whycon yujie_whycon_rviz.launch
  # without RVIZ
  roslaunch whycon yujie_whycon.launch
  ```

- **using rosbag**

  ```shell
  roslaunch whycon yujie_whycon_rviz_rosbag.launch
  roslaunch whycon yujie_whycon_rviz_rosbag.launch bag_name:=whycon_short_0311
  ```

#### summary and experiments

- its robustness and accuracy is not sensitive regarding the resolution and the focal length of the camera

- valid from 0.2m to more than 6m when using whycon marker with largest diameter is about square size = 0.200m on A4 paper and the camera with 640x480 resolution

- **Efficiency summary: If the target is tracked continuously, the whycon algorithm runs extremely fast, more than 2000fps, if the target is lost, it will take extra time to find the target using the whole image, running at the speed of about 50-100 fps.**

- **can run as a mavconn service, outputting pose information through bus**

- :construction: Warning

  - remember to remap topics in ROS!
  - don't worry when you get warning!!!
- some function get changed in OpenCV4, such as `CV_AA` to `cv.LINE_AA`

- ⭐change text color in visualization in `circle_detector.cpp`

### aruco_yujie

> modified from <https://github.com/CesMak/aruco_detector_ocv.git>

- installation

  ```c++
  // first introduced in OpenCV 3.3.0
  // https://docs.opencv.org/3.2.0/d9/d6a/group__aruco.html
  // https://docs.opencv.org/3.3.0/d9/d6a/group__aruco.html
  // https://stackoverflow.com/questions/56318165/error-struct-cvarucodetectorparameters-has-no-member-named-cornerrefinem
  // detector_params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
  ```

#### Summary

- valid between 0.35m and 3.3m when using 5x5 aruco marker with square size = 0.178m on an A4 paper and the camera with 640x480 resolution
- its robustness is highly dependant on the resolution and the focal length of the camera

#### command line

- **run the code**

  ```shell
  # change device depends on connection
  roslaunch cv_camera cv_camera_only.launch device_id:=1
  # with RVIZ
  roslaunch aruco_yujie aruco_yujie_5_5.launch
  ```

- **using rosbag**

  ```shell
  roslaunch aruco_yujie aruco_yujie_5_5_rosbag.launch
  ```

### aruco_ros

- pal-robotics/aruco_ros: <https://github.com/pal-robotics/aruco_ros>

### others

#### official

- [ros-perception/**vision_opencv**](https://github.com/ros-perception/vision_opencv)

  ```shell
  git clone -b melodic https://github.com/ros-perception/vision_opencv.git
  ```

- [image_proc](http://wiki.ros.org/image_proc)

#### deprecated

- [warp1337/**ros_aruco**](https://github.com/warp1337/ros_aruco)
- [gestom/**whycon-orig**](https://github.com/gestom/whycon-orig/tree/opencv)
- [AprilRobotics/**apriltag_ros**](https://github.com/gestom/whycon-orig/tree/opencv)
  can only be compiled using `catkin_build`
