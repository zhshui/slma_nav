#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/unitree/go2_nav/lite_cog/drivers/mid360_ws/devel/setup.bash --extend

roslaunch /home/unitree/go2_nav/lite_cog/drivers/mid360_ws/src/livox_ros_driver2-master/launch_ROS1/rviz_MID360.launch
