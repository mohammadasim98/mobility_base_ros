#!/bin/sh
rosparam dump `date +%Y-%m-%d-%H-%M-%S`.yaml
rosbag record /mobility_base/bumper_states /mobility_base/imu/data_raw /mobility_base/imu/mag /mobility_base/joint_states /mobility_base/joystick /mobility_base/mode /mobility_base/twist /mobility_base/wrench /laser_birdcage_vlp16/points /laser_birdcage_r2000/scan /laser_front/scan /laser_rear/scan /tf /tf_static
