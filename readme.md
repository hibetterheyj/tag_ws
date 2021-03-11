# Marker detection

## Work with camera

### shell

- list cameras and its permissions
  
  - `ll /dev/video*`
- cannot access camare
  
  - `chmod 777 /dev/video1`

### guvcview

- sudo apt install guvcview
    1. Check the available resolutions, framerates and formats of your USB camera by running

       ```shell
       guvcview --device=/dev/video1
       ```

    2. Run guvcview and adjust your camera settings (exposure, brightness etc).

---

## ROS package

### cv_camera

- launch files
  - calibrate

    ```shell
    roslaunch cv_camera cv_camera_only.launch
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

- parameters
  - `~camera_info_url (string)` – url of camera info yaml.

- rostopic published by cv_camera

### image_proc

- http://wiki.ros.org/image_proc
- https://answers.ros.org/question/208724/how-to-run-image_proc-with-roslaunch/

### vision_opencv

```shell
git clone https://github.com/ros-perception/vision_opencv.git
git checkout melodic
```

then change the `CMakeLists.txt` by setting specfic version, such as `find_package(OpenCV 3.4 REQUIRED`

### whycon

- code: https://github.com/lrse/whycon
- wiki: https://github.com/lrse/whycon/wiki
  - [Reference: A description of the ROS nodes can be found here](https://github.com/lrse/whycon/wiki/Reference)
  - [Usage Tutorial: See a step-by-step tutorial here.](https://github.com/lrse/whycon/wiki/Tutorial)

- **run the code**

  ```shell
  # change device depends on connection
  roslaunch cv_camera cv_camera_only.launch device_id:=1
  roslaunch whycon yujie_whycon_rviz.launch
  ```

- **using rosbag**

  ```shell
  roslaunch whycon yujie_whycon_rviz.launch
  rosbag play -r 0.8 <whycon_xxx.bag>
  ```
  

- note
  - remember to remap topics in ROS!
  - don't worry when you get warning!!!
  - some function get changed in OpenCV4, such as `CV_AA` to `cv.LINE_AA`

### aruco_ros

- pal-robotics/aruco_ros: <https://github.com/pal-robotics/aruco_ros>

### deprecated

#### ros_aruco (deprecated)

- warp1337/ros_aruco: <https://github.com/warp1337/ros_aruco>

#### whycon-orig (deprecated)

- code: https://github.com/gestom/whycon-orig/tree/opencv

## Misc.

- apriltag only works when using `catkin build` https://github.com/AprilRobotics/apriltag_ros
- ⭐ add delay time in launch files

  ```xml
  <arg name="node_start_delay" default="1.0" />  
  <node name="listener" pkg="roscpp_tutorials" type="listener" launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@' " />
  ```
  