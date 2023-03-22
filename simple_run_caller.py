"""
example file for running ros2
"""

from ros2_easy.classes.Package import *
from ros2_easy.classes.launch_configuration import *
from ros2_easy.classes.common_packages_and_programs import *

import yaml
import ros2_easy.simple_run as simple_run
import os

model_pkg = Package("model_pkg", "model", build=True, entry_point="main")

rviz2_config_name = "rviz_config_test.rviz"
rviz2_pkg = Package("rviz2", "rviz2", config=Config(config_file_name=rviz2_config_name), optional_launch_file_node_args= {"arguments": "['-d', share_directory + '/rviz/%s']" % rviz2_config_name})


rviz_env_conf = launch_configuration(
    config_store_pkg=model_pkg,
    launch_file="rviz_config_test_launch.py",
    urdf_file_name="diff_bot",
    packages_to_run=[model_pkg, rviz2_pkg, rqt_pkg, robot_state_publisher_pkg],
    )
"""This launch configuration launches rviz alone. Use this when you want to see what a physical robot is doing"""

env_to_use = rviz_env_conf
"""launch conf to use"""

"""
model file which is used by urdf generator.
Checks for this inside the /models folder for the 'config_store_pkg' for the used luanch_configuration

"""


#simple_run.replace_setup_py(env_to_use)
#simple_run.generate_launch_py(env_to_use)
#simple_run.create_urdf_of_model(env_to_use)
#simple_run.construct_bash_script(env_to_use)
simple_run.launch_gazebo_world(env_to_use)