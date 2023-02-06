
import os
import subprocess
import glob
import sys
import typing
import json
import trimesh
import logging
import inspect
from dataclasses import dataclass
from typing import Optional

from .Package import Package
from .Cmd_Program import Cmd_Program
from .Config import Config
from .logger import return_logger


@dataclass
class launch_configuration():
    """
    object that holds all of the parameters for a specific type of a ros2 enviorment. E.G: configuration for running a simulation enviorment/real enviorment.
    """
    config_store_pkg: Package
    """
    package which stores all configs.
    
    This includes things like launch files, rviz configs, etc..

    for sanity sake the only option will be to store all configs in 1 package.
    """
    launch_file: str
    """The full name of the launch file for this launch configuration. Include the .py extension. E.G:
        
        `launch.py`
    """
    urdf_file: str
    """Full name of the urdf file. This should be either a pre-made file or the name of the final generated file from the create_urdf function"""

    packages_to_run: typing.List[Package]
    """All packages to be ran by this launch configuration"""
    extra_pkgs_to_build: Optional[typing.List[Package]] = None
    """The packages to be built by colcon in addition to config storing pkg"""
    external_programs_to_run: Optional[typing.List[Cmd_Program]] = None
    """External programs to run that aren't ROS2 packages. E.G: Gazebo/Ignition"""
