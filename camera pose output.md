### Control_strategy

#### Overview

- key information for vertical marker detection
  - important values: **x/y relative to marker, distance to marker, heading towards marker**, pitch/yaw
  - X, Y, Dist, Yaw value

- modes
  - takeoff mode
  - position mode
  - offboard mode

#### [TODO] How to do visual servoing with Parrot Bebop 2 drone using visp_ros

> https://wiki.ros.org/visp_ros/Tutorials/How%20to%20do%20visual%20servoing%20with%20Parrot%20Bebop%202%20drone%20and%20visp_ros
>
> https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-bebop2-vs.html

- bebop_autonomy: https://github.com/lagadic/bebop_autonomy

- BebopS: https://github.com/gsilano/BebopS

- Parrot-Sphinx: https://developer.parrot.com/docs/sphinx/whatissphinx.html

  compellable with BebopS

- code: https://github.com/lagadic/visp/tree/master/example/servo-bebop2

#### [TODO] The Urban Last Mile Problem: Autonomous Drone Delivery to Your Balcony

> [code](https://github.com/szebedy/autonomous-drone)

- https://github.com/szebedy/autonomous-drone/blob/master/src/offboard_control/main.cpp

  ```c++
  /**
   * @file main.cpp
   * @brief offboard control node, written with mavros version 0.26.0,
   * px4 flight stack 1.8.0 and tested on Intel Aero RTF and in Gazebo SITL
   */
  
  #include "include/drone_control.h"
  #include "include/ros_client.h"
  
  int main(int argc, char **argv)
  {
    ROSClient ros_client(argc, argv);
    DroneControl drone_control(&ros_client);
  
    drone_control.offboardMode();
  
    drone_control.vioOff();
  
    drone_control.takeOff();
  
    drone_control.flyToLocal(4.0, -5.0, DroneControl::SAFETY_ALTITUDE_GPS);
    //drone_control.initVIO();
  
    drone_control.collisionAvoidOn();
  
    drone_control.scanBuilding();
  
    drone_control.centerMarker();
    // hover
    drone_control.hover(10);
  
    drone_control.approachMarker();
    // hover
    drone_control.hover(10);
    // send landing command at current position
    drone_control.land();
    // stop all machines
    drone_control.disarm();
  
    while(ros::ok() && DroneControl::KEEP_ALIVE)
    {
      ros::spin();
    }
  
    return 0;
  }
  ```

- https://github.com/szebedy/autonomous-drone/blob/master/src/trajectory_planner/src/trajectory_planner.cpp

- https://github.com/szebedy/autonomous-drone/blob/master/src/offboard_control/drone_control.cpp

  ```c++
  // void DroneControl::turnTowardsMarker()
  void DroneControl::centerMarker()
  void DroneControl::approachMarker()
  ```

---

#### [Challenges and Implemented Technologies Used in Autonomous Drone Racing](http://rpg.ifi.uzh.ch/docs/ISR19_Moon.pdf)

- **Team INAOE**
  - The control was implemented with 3 controllers based on the PID (Proportional-Integral-Differential) controller to control height, heading (yaw controller) and forward/sideways motion (pitch and roll controller), trigger in that order once the drone reached a waypoint, current height and yaw was measured with the drone’s altitude and IMU sensors
  -  The reason why the drone took more time in its flight was due to the yaw controller, which spent considerable time in reaching the yaw reference, for instance, right at the outset, the controller spent 25 seconds to reach the first yaw reference
- **UZH**
  - The high-level controller receives as input the reference position, velocity, acceleration and yaw, and produces the desired collective thrust and body rates. These are sent to the low-level controller, which is responsible for body rate control (i.e., transforms the reference body rates into desired torques) and computes the single-rotor thrusts necessary to achieve the reference collective thrust and torques.

---

### camera pose output

> :construction: **pose representation !!!**

- ZYX: ψ为Yaw偏航角，θ为Roll俯仰角，φ为Pitch滚转角

- xyz: camera_relative to marker

- fractal_ros

  ```
        A (y)
        |
        |
  <-----|
  (x)
  ```

  - fractal_cvcam_right_down

    marker on the left/up, camera on the right/down, both x/y as negative

  - fractal_cvcam_left_up

    marker on the right/down, camera on the left/up, both x/y as positive

  yaw left to center to right: <90 -> 90 -> >90 degrees

- whycon

  ```
        A (y)
        |
        |
  <-----|
  (x)
  ```

  - whycon_cvcam_right_down

    marker on the left/up, camera on the right/down, both x/y as negative

    :construction: degrees are not very stable

- new version of whycon: https://github.com/jiriUlr/whycon-ros

  ```
  float angle;                // axis angle around marker's surface normal
  ```

- aruco marker
- apriltag

---

### convert eular angles from qua

- tf: https://docs.ros.org/en/diamondback/api/tf/html/c++/namespacetf.html

  ```c++
  static geometry_msgs::Quaternion tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw)
  ```

- [**Quaternion to Euler angle convention in TF**](https://answers.ros.org/question/239689/quaternion-to-euler-angle-convention-in-tf/)

  ```c++
  //Let q be the quaternion of the current odom transform
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3 mat(q);
  mat.getEulerYPR(&yaw, &pitch, &roll);
  ```

- https://github.com/szebedy/autonomous-drone/blob/master/src/offboard_control/drone_control.cpp

  ```c++
  double DroneControl::getYaw(const geometry_msgs::Quaternion &msg)
  {
    //Calculate yaw current orientation
    double roll, pitch, yaw;
    tf::Quaternion q;
  
    tf::quaternionMsgToTF(msg, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  
    return yaw;
  }
  ```

  