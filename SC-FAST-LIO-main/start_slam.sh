#!/bin/bash
# SLAM start script for SC-FAST-LIO mapping_mid360

export DISPLAY=${DISPLAY:-:0}

source /opt/ros/noetic/setup.bash
source /home/unitree/go2_nav/lite_cog/drivers/mid360_ws/devel/setup.bash
source /home/unitree/go2_nav/SC-FAST-LIO-main/devel/setup.bash

roslaunch fast_lio mapping_mid360.launch "$@"
