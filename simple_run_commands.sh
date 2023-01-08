#!/bin/bash


rm -r build install

colcon build --packages-select model_pkg rviz2 rqt_gui robot_state_publisher

source /opt/ros/foxy/setup.bash

source install/setup.sh

ros2 launch model_pkg  rviz_config_test_launch.py