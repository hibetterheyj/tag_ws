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

       ```
       guvcview --device=/dev/video1
       ```

    2. Run guvcview and adjust your camera settings (exposure, brightness etc).

---

## ROS package

### cv_camera

- rostopic published by cv_camera
  ```
  /cv_camera/camera_info
  /cv_camera/image_raw
  /cv_camera/image_raw/compressed
  /cv_camera/image_raw/compressed/parameter_descriptions
  /cv_camera/image_raw/compressed/parameter_updates
  /cv_camera/image_raw/compressedDepth
  /cv_camera/image_raw/compressedDepth/parameter_descriptions
  /cv_camera/image_raw/compressedDepth/parameter_updates
  /cv_camera/image_raw/theora
  /cv_camera/image_raw/theora/parameter_descriptions
  /cv_camera/image_raw/theora/parameter_updates
  /rosout
  /rosout_agg
  ```

```shell
roslaunch cv_camera cv_camera_test.launch
roslaunch cv_camera cv_camera_test.launch device_id:=1
```

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

### whycon-orig (deprecated)
- code: https://github.com/gestom/whycon-orig/tree/opencv

```
cd src/
make CXX=clang
# Error
```