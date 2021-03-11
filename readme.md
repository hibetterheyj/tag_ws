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
  # with RVIZ
  roslaunch whycon yujie_whycon_rviz.launch
  # without RVIZ
  roslaunch whycon yujie_whycon.launch
  ```

- **using rosbag**

  ```shell
  roslaunch whycon yujie_whycon_rviz.launch
  rosbag play -r 0.8 <whycon_xxx.bag>
  ```

- Note

  - publish function in `whycon_ros.cpp`

    ```c++
    void whycon::WhyConROS::publish_results(const std_msgs::Header& header, const cv_bridge::CvImageConstPtr& cv_ptr)
    ```

  - tracking and count fps in `circle_detector.cpp`

    ```c++
    int64_t ticks = cv::getTickCount();  
    
    ...
        
    double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
      cout << "examineCircle: " << delta << " " << " fps: " << 1/delta << " pix: " << circle.size << " " << threshold << endl;
    ```

    ---

    - **Efficiency summary: If the target is tracked continuously, the whycon algorithm runs extremely fast, more than 2000fps, if the target is lost, it will take extra time to find the target using the whole image, running at the speed of about 50-100 fps.**

    - speed when examining Circle

      - `examineCircle(image, outer, ii, outerAreaRatio, search_in_window)`

      - results

        ```
        examineCircle: 8.91499999999999985598325702441e-05  fps: 11217.0499158721249841619282961 pix: 749 490
        examineCircle: 2.08749999999999991877157323739e-05  fps: 47904.1916167664676322601735592 pix: 215 490
        examineCircle: 9.25059999999999982766771267073e-05  fps: 10810.1096145114915998419746757 pix: 766 493
        examineCircle: 2.51289999999999994547295739666e-05  fps: 39794.6595566874966607429087162 pix: 211 493
        examineCircle: 9.94279999999999973938138997376e-05  fps: 10057.5290662590014107991009951 pix: 774 492
        examineCircle: 2.40820000000000008393615663627e-05  fps: 41524.7902998089848551899194717 pix: 214 492
        ```

    - overall speed: 

      - implementation

        ```
          // yujie0311
          int64_t ticks = cv::getTickCount();
        
          is_tracking = system->localize(image, should_reset/*!is_tracking*/, max_attempts, max_refine);
        
            double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
            cout << "checking circle and compute distance: " << delta << " " << " fps: " << 1/delta << endl;
        ```

      - results

        ```
        checking circle and compute distance: 0.000344053999999999996113608791148  fps: 2906.52048806292032168130390346
        checking circle and compute distance: 0.000346080999999999981957959915846  fps: 2889.49696747293273801915347576
        checking circle and compute distance: 0.000347040000000000011231432450742  fps: 2881.51221761180249814060516655
        checking circle and compute distance: 0.0199971720000000005579288142599  fps: 50.0070709998393780892911308911
        checking circle and compute distance: 0.021584013999999998589807148619  fps: 46.3305852192275295919898780994
        checking circle and compute distance: 0.00624448899999999976900832976412  fps: 160.141206109899457032952341251
        checking circle and compute distance: 0.00618661599999999981674969617984  fps: 161.639254804241943475062726066
        checking circle and compute distance: 0.00670624900000000006422551379615  fps: 149.114654108429306234029354528
        checking circle and compute distance: 0.0221067970000000009467289174836  fps: 45.2349564706275586445372027811
        checking circle and compute distance: 0.0131158100000000003154676520012  fps: 76.2438614161077339304029010236
        checking circle and compute distance: 0.000383309999999999987535526102533  fps: 2608.85445201012225879821926355
        checking circle and compute distance: 0.0155065110000000005996412255627  fps: 64.4890394750953248603764222935
        ```

  - **can run as a mavconn service, outputting pose information through bus**

- :construction: Warning
  
  - remember to remap topics in ROS!
  - don't worry when you get warning!!!
- some function get changed in OpenCV4, such as `CV_AA` to `cv.LINE_AA`
  
- ⭐change text color in visualization

  - circle_detector.cpp
  
    ```c++
    //  yujie0311
    //  float scale = image.size().width / 1800.0f;
    float scale = image.size().width / 800.0f;
    cv::Scalar color_ = cv::Scalar(0, 0, 0);
    //float thickness = scale * 3.0;
    //if (thickness < 1) thickness = 1;
    cv::putText(image, text.c_str(), cv::Point(x + 2 * m0, y + 2 * m1), CV_FONT_HERSHEY_SIMPLEX, scale, cv::Scalar(color_), 2, CV_AA);
    cv::line(image, cv::Point(x + v0 * m0 * 2, y + v1 * m0 * 2), cv::Point(x - v0 * m0 * 2, y - v1 * m0 * 2), cv::Scalar(color_), 2, 8);
    cv::line(image, cv::Point(x + v1 * m1 * 2, y - v0 * m1 * 2), cv::Point(x - v1 * m1 * 2, y + v0 * m1 * 2), cv::Scalar(color_), 2, 8); 
    ```

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

## Problems

- **[fixed]** [ros-perception/image_common/camera_info_manager/src/camera_info_manager.cpp](https://github.com/ros-perception/image_common/blob/noetic-devel/camera_info_manager/src/camera_info_manager.cpp)

  Question: If I set a custom path such as src/cv_camera/config/camera.yaml, I will encounter this error

    ```shell
    [ERROR] [1615434795.397894710]: cv camera open failed: file /home/he/Desktop/TagDection/tag_ws/devel/share/cv_camera/../../../src/cv_camera/config/trust_webcam/camera.yaml cannot be opened
    [cv_camera-1] process has died [pid 1973, exit code 1, cmd /home/he/Desktop/TagDection/tag_ws/devel/lib/cv_camera/cv_camera_node __name:=cv_camera __log:=/home/he/.ros/log/a69d5f7c-821b-11eb-bb8a-f0038c0ab1cd/cv_camera-1.log].
    log file: /home/he/.ros/log/a69d5f7c-821b-11eb-bb8a-f0038c0ab1cd/cv_camera-1*.log
    ```

  - fix in two ways
    1. `cp src/cv_camera/config/camera.yaml ~/.ros/camera_info/camera.yaml`
    2. writing the following command in the launch file

    ```xml
    <arg name="calib_file_path" default="file://$(find cv_camera)/config/trust_webcam/camera.yaml" />
    ```
