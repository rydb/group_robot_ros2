#!/bin/bash


rm -r build install

colcon build --packages-select model_pkg

source /opt/ros/foxy/setup.bash

source install/setup.sh

ros2 launch model_pkg  sim_launch.py