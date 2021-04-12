echo "in tag_ws/"
cd tag_ws/
source devel/setup.bash
echo $ROS_PACKAGE_PATH
cd ..
cd Desktop/
cd MYNT-EYE-S-SDK/
echo "in MYNT-EYE-S-SDK/"
rm -rf ./wrappers/ros/devel/
make ros
source wrappers/ros/devel/setup.bash 
echo $ROS_PACKAGE_PATH
cd ~
