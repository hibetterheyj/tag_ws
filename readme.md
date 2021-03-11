# Marker detection

⭐ some huge files can be found [here](https://drive.google.com/drive/folders/1qwV2kRVkfSNCs9ddREWbypiysS995Kv9?usp=sharing)

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

### [image_proc](http://wiki.ros.org/image_proc)

### vision_opencv

```shell
git clone https://github.com/ros-perception/vision_opencv.git
git checkout melodic
```

then change the `CMakeLists.txt` by setting specfic version, such as `find_package(OpenCV 3.4 REQUIRED`

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

#### summary and experiments

- its robustness and accuracy is not sensitive regarding the resolution and the focal length of the camera

- valid from 0.2m to more than 6m when using whycon marker with largest diameter is about square size = 0.200m on A4 paper and the camera with 640x480 resolution

- **Efficiency summary: If the target is tracked continuously, the whycon algorithm runs extremely fast, more than 2000fps, if the target is lost, it will take extra time to find the target using the whole image, running at the speed of about 50-100 fps.**

- overall speed

  - implementation

    ```c++
    // yujie0311
    int64_t ticks = cv::getTickCount();
  
    is_tracking = system->localize(image, should_reset/*!is_tracking*/, max_attempts, max_refine);
  
    double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
    cout << "checking circle and compute distance: " << delta << " " << " fps: " << 1/delta << endl;
    ```

  - results: 
    - **personal computer** Intel® Core™ i7-6700HQ CPU @ 2.60GHz × 8: more than 50 Hz when finding whycon and more than 2000 Hz when keeping tracking
    - **student room computer** Intel® Core™ i7-6700 CPU @ 3.40GHz × 8: more than 200 Hz when finding whycon and more than 10000 Hz when keeping tracking

    ```shell
    # personal
    checking circle and compute distance: 0.000344053999999999996113608791148  fps: 2906.52048806292032168130390346
    checking circle and compute distance: 0.000346080999999999981957959915846  fps: 2889.49696747293273801915347576
    checking circle and compute distance: 0.000347040000000000011231432450742  fps: 2881.51221761180249814060516655
    checking circle and compute distance: 0.0199971720000000005579288142599  fps: 50.0070709998393780892911308911
    checking circle and compute distance: 0.021584013999999998589807148619  fps: 46.3305852192275295919898780994
    checking circle and compute distance: 0.00624448899999999976900832976412  fps: 160.141206109899457032952341251
    checking circle and compute distance: 0.00618661599999999981674969617984  fps: 161.639254804241943475062726066
    # student room
    checking circle and compute distance: 0.00479298200000000031856250970463  fps: 208.638380031471001530007924885
    checking circle and compute distance: 0.00158503099999999998333255479821  fps: 630.902487080694299947936087847
    checking circle and compute distance: 0.00564151199999999956868901307416  fps: 177.25744445815237781971518416
    checking circle and compute distance: 0.00550736699999999999161071073672  fps: 181.574970398740447308227885514
    checking circle and compute distance: 4.65889999999999972766021039128e-05  fps: 21464.294146686988824512809515
    checking circle and compute distance: 7.81700000000000054306212304844e-05  fps: 12792.6314442880884598707780242
    checking circle and compute distance: 7.31769999999999941125636282457e-05  fps: 13665.4959891769285604823380709
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

- Note

  - tracking and count fps in `circle_detector.cpp`

    ```c++
    int64_t ticks = cv::getTickCount();

    // Smooth the image to improve detection results
    if (enable_blur) {
        GaussianBlur(image, image, Size(blur_window_size, blur_window_size), 0,
                     0);
    }

    // Detect the markers
    vector<int> ids;
    vector<vector<Point2f>> corners, rejected;
    aruco::detectMarkers(image, dictionary, corners, ids, detector_params, rejected);

    double delta = (double)(cv::getTickCount() - ticks) / cv::getTickFrequency();
    cout << "checking aruco: " << delta << " " << " fps: " << 1/delta << endl;
    ```

  - results: 
    - **personal computer** Intel® Core™ i7-6700HQ CPU @ 2.60GHz × 8: about 170-260Hz (with some low points)
    - **student room computer** Intel® Core™ i7-6700 CPU @ 3.40GHz × 8: about 190-290Hz 

    ```shell
    # personal
    checking aruco: 0.0047509  fps: 210.486   Duration: 7.765050 / 18.342437               
    checking aruco: 0.00424327  fps: 235.667  Duration: 7.800998 / 18.342437
    # some low points here
    checking aruco: 0.0114855  fps: 87.0665   Duration: 7.844573 / 18.342437               
    checking aruco: 0.015256  fps: 65.54829   Duration: 7.875542 / 18.342437               
    checking aruco: 0.00584501  fps: 171.086  Duration: 7.908486 / 18.342437               
    checking aruco: 0.00435351  fps: 229.699  Duration: 7.941409 / 18.342437               
    checking aruco: 0.00375162  fps: 266.551  Duration: 7.972672 / 18.342437               
    checking aruco: 0.0054297  fps: 184.172   Duration: 8.008810 / 18.342437               
    checking aruco: 0.00457636  fps: 218.514  Duration: 8.044999 / 18.342437               
    checking aruco: 0.00379929  fps: 263.207  Duration: 8.080643 / 18.342437               
    checking aruco: 0.00415778  fps: 240.513  Duration: 8.112967 / 18.342437
    # student room        
    checking aruco: 0.00437748  fps: 228.442  Duration: 7.765037 / 18.342437               
    checking aruco: 0.00358874  fps: 278.649  Duration: 7.800346 / 18.342437               
    checking aruco: 0.00382045  fps: 261.749  Duration: 7.836576 / 18.342437               
    checking aruco: 0.00346087  fps: 288.944  Duration: 7.868440 / 18.342437               
    checking aruco: 0.00360971  fps: 277.03   Duration: 7.904529 / 18.342437               
    checking aruco: 0.00411221  fps: 243.178  Duration: 7.944021 / 18.342437               
    checking aruco: 0.00354886  fps: 281.781  Duration: 7.972599 / 18.342437               
    checking aruco: 0.00376028  fps: 265.938  Duration: 8.008783 / 18.342437               
    checking aruco: 0.00533363  fps: 187.489  Duration: 8.049162 / 18.342437               
    checking aruco: 0.00486725  fps: 205.455  Duration: 8.081169 / 18.342437               
    checking aruco: 0.00531744  fps: 188.06   Duration: 8.112955 / 18.342437               
    ```

#### command line

- **run the code**

  ```shell
  # change device depends on connection
  # roslaunch cv_camera cv_camera_view_rect.launch
  roslaunch cv_camera cv_camera_only.launch
  # with RVIZ
  roslaunch aruco_yujie aruco_yujie_5_5.launch
  ```

- **using rosbag**

  ```shell
  roslaunch aruco_yujie aruco_yujie_5_5_rosbag.launch
  ```

### aruco_ros

- pal-robotics/aruco_ros: <https://github.com/pal-robotics/aruco_ros>

### deprecated

#### ros_aruco (deprecated)

- warp1337/ros_aruco: <https://github.com/warp1337/ros_aruco>

#### [whycon-orig (deprecated)](https://github.com/gestom/whycon-orig/tree/opencv)

## Misc

- apriltag only works when using `catkin build` <https://github.com/AprilRobotics/apriltag_ros>
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
