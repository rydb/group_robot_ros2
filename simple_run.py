#Constructs a bash script to source, compile, remove relics, etc.. and run a user defined set of packages.

import os
import subprocess
import glob
import sys
import typing
import json
#import open3d
import trimesh

from dataclasses import dataclass
from typing import Optional

from classes.Package import Package
from classes.Cmd_Program import Cmd_Program
from classes.Config import Config

freecad_macros_folder_name = "freecad_macros"
model_file_name = "urdfmodel.FCStd"
"""name of model file"""
PROJECT_DIRECTORY = "/home/rydb/Projects/group_robot_ros2"
"""root directory of ros2 project. should make this not hard coded later"""
print("Hi!")


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


gazebo = Cmd_Program("gazebo.gz gazebo")
"""use:
    sudo snap install --beta gazebo
    to install gazebo"""
model_pkg = Package(None, "model_pkg", "model", build=True)
#model_pkg.config = Config(model_pkg.folder_path())

rviz2_config_name = "rviz_config_test.rviz"
rviz2_pkg = Package(model_pkg, "rviz2", "rviz2", config=Config(config_file_name=rviz2_config_name), optional_launch_file_node_args= {"arguments": "['-d', share_directory + '/rviz/%s']" % rviz2_config_name})

robot_state_publisher_pkg = Package(model_pkg, "robot_state_publisher", "robot_state_publisher", optional_launch_file_node_args= {"parameters": "[{'use_sim_time': True, 'robot_description': robot_desc}]"})
rqt_pkg = Package(model_pkg, "rqt_gui", "rqt_gui")



real_rviz_env_conf = launch_configuration(
    config_store_pkg=model_pkg,
    launch_file="rviz_config_test_launch.py",
    urdf_file="diff_bot.urdf.xml",
    packages_to_run=[model_pkg, rviz2_pkg, rqt_pkg, robot_state_publisher_pkg],
    )
"""This launch configuration launches rviz alone. Use this when you want to see what a physical robot is doing"""

gazebo_env_conf = launch_configuration(
    config_store_pkg=model_pkg,
    launch_file="gazebo_config_test_launch.py",
    urdf_file="diff_bot.urdf.xml",
    packages_to_run=[model_pkg, rqt_pkg],
    external_programs_to_run=[gazebo],
    )
"""This launch configuration launches gazebo. Use this for physics simulation"""


def replace_setup_py(launch_conf: launch_configuration):
    """
    adds EVERY folder in each package_to_build to their /share directorys for each setup.py

    ROS2 requires everything it uses to be in the /share directory, so for the sake of saving headaches, pre-emptively add EVERYTHING to it. 
    """
    
    tab = "    "

    directory_of_file = "src/%s/" % launch_conf.config_store_pkg.name
    file_to_edit = "setup.py"
    print(".%s*/" % directory_of_file)
    setup_py = directory_of_file + file_to_edit
    print("re-writting ||%s|| to include all folders inside them inside their /install/<pkg_name>/share directory" % launch_conf.config_store_pkg.name)

    f = open(setup_py, "w")
    sample_f_list = []

    sample_f_list.append(['WRITE', 0, "from setuptools import setup"])
    sample_f_list.append(['WRITE', 0, "import os"])
    sample_f_list.append(['WRITE', 0, "import glob"])
    sample_f_list.append(['WRITE', 0, " "])
    sample_f_list.append(['WRITE', 0, "#NOTE: MAKE SURE YOUR LAUNCH FILE IS INSIDE <pkg_name>/launch"])
    sample_f_list.append(['WRITE', 0, " "])
    sample_f_list.append(['WRITE', 0, "#GLOB STARTS FROM THE PACKAGE DIRECTORY WHEN USING setup.py"])
    sample_f_list.append(['WRITE', 0, " "])
    sample_f_list.append(['WRITE', 0, "package_name = 'model_pkg'"])
    sample_f_list.append(['WRITE', 0, " "])
    sample_f_list.append(['WRITE', 0, "setup("])
    \
        sample_f_list.append(['WRITE', 1, "name=package_name,"])
    \
        sample_f_list.append(['WRITE', 1, "version='0.0.0',"])
    \
        sample_f_list.append(['WRITE', 1, "packages=[package_name],"])
    \
        sample_f_list.append(['WRITE', 1, "data_files=["])
    \
            sample_f_list.append(['WRITE', 2, "('share/ament_index/resource_index/packages',"])
    \
                sample_f_list.append(['WRITE', 3, "['resource/' + package_name]),"])
    \
            sample_f_list.append(['WRITE', 2, "('share/' + package_name, ['package.xml']),"])
    \
                    for folder in [dire[dire.__len__() - 2]for dire in [dire.split("/") for dire in glob.glob("./src/model_pkg/*/", recursive = True)]]:
                        #append every folder in package into /share so files don't get loaded because you forget xyz
                        print("writting %s" %folder)
                        sample_f_list.append(["WRITE", 4, "(os.path.join('share', package_name, '%s'), glob.glob('%s/*'))," % (folder, folder) ])
    \
        sample_f_list.append(['WRITE', 1, "],"])
    \
        sample_f_list.append(['WRITE', 1, "install_requires=['setuptools'],"])
    \
        sample_f_list.append(['WRITE', 1, "zip_safe=True,"])
    \
        sample_f_list.append(['WRITE', 1, "maintainer='insert_here',"])
    \
        sample_f_list.append(['WRITE', 1, "maintainer_email='placeholder@gmail.com',"])
    \
        sample_f_list.append(['WRITE', 1, "description='TODO: Package description',"])
    \
        sample_f_list.append(['WRITE', 1, "license='TODO: License declaration',"])
    \
        sample_f_list.append(['WRITE', 1, "tests_require=['pytest'],"])
    \
        sample_f_list.append(['WRITE', 1, "entry_points={"])
    \
            sample_f_list.append(['WRITE', 2, "'console_scripts': ["])
    \
                sample_f_list.append(['WRITE', 3, "'model = model_pkg.model:main'"])
    \
            sample_f_list.append(['WRITE', 2, "],"])
    \
        sample_f_list.append(['WRITE', 1, "},"])
    \
    sample_f_list.append(['WRITE', 0, ")"])

    print(setup_py)
    f = open(setup_py, 'r')
    f_lines = f.readlines()
    f.close()
    f = open(setup_py, 'w')
    tab = "    "
    for i in range(0, sample_f_list.__len__()):
        l =  sample_f_list[i]
        if(l[0] == 'IGNORE'):
            f.write(f_lines[i])
        elif(l[0] == 'WRITE'):
            f.write((tab * l[1]) + l[2] + "\n")
        else:
            print('WARNING: Expected IGNORE or WRITE, got ' + l[0] + 'treating as IGNORE')
            f.write(f_lines[i])

def generate_launch_py(launch_conf: launch_configuration, ):
    """
    managing launch files is too much mental micromangement, so instead generate launch file to run based on launch_configuration here.

    collects:
        1: all packages that are being built, 
        2: launch file directory, 
        3: misc launch variables that are in launch_configuration

    and then creates a launch.py file based off of those parameters for other functions in simple_run to launch.
    """

    #fetch share directory command, since this needs to be executed as a part of the launch file, append the command to be executed inside the launch file
    package_name_var_name = "package_name"
    share_directory_command = "ament_index_python.packages.get_package_share_directory(%s)" % package_name_var_name

    screen = "'screen'"
    ###
    # Packages/commands that go into launch file
    ###
    launch_commands = []

    """
    if gazebo fails to load, run the following commands(if on ubuntu) to install missing packages
    https://classic.gazebosim.org/tutorials?tut=ros2_installing

    sudo apt install ros-foxy-ros-core ros-foxy-geometry2
    sudo snap install gazebo

    launch setup based on:
    https://answers.ros.org/question/374976/ros2-launch-gazebolaunchpy-from-my-own-launch-file/
    """

    #get every line ready and constructed to be written into the new launch file
    launch_l_list = []


    #print(launch_conf.packages_to_run)

    f = open("./src/%s/launch/%s" % (launch_conf.config_store_pkg.name, launch_conf.launch_file), "w")
    launch_l_list = []

    launch_l_list.append([0, "from launch import LaunchDescription"])
    launch_l_list.append([0, "from launch_ros.actions import Node"])
    launch_l_list.append([0, "from launch.actions import ExecuteProcess"])
    launch_l_list.append([0, "import ament_index_python\n"])

    launch_l_list.append([0, "package_name = 'model_pkg'"])
    launch_l_list.append([0, "share_directory = ament_index_python.packages.get_package_share_directory(%s)\n" % package_name_var_name])
    launch_l_list.append([0, "urdf = share_directory + '/urdf/%s'" % launch_conf.urdf_file])

    launch_l_list.append([ 0, "def generate_launch_description():"])
    \
        launch_l_list.append([1, "with open(urdf, 'r') as infp:"])
    \
            launch_l_list.append([2, "robot_desc = infp.read()\n\n"])
    \
        launch_l_list.append([ 1, "return LaunchDescription(["])
    \
        if(launch_conf.external_programs_to_run != None):
            for external_prog in launch_conf.external_programs_to_run:
                launch_l_list.append([2, "ExecuteProcess("])

                program_args = external_prog.as_process_conf_dict()
                for param in program_args:
                    launch_l_list.append([3, "%s=%s," % (param, program_args[param])])
                launch_l_list.append([2, "),"])
    \
        for pkg in launch_conf.packages_to_run:

            launch_l_list.append([2, "Node("])

            node_args = pkg.as_node_conf_dict()
            for param in node_args:
                launch_l_list.append([3, "%s=%s," % (param, node_args[param])])
            launch_l_list.append([ 2, "),"])
    \
        launch_l_list.append([ 1, "])"])

    tab = "    "
    for i in range(0, launch_l_list.__len__()):
        l =  launch_l_list[i]
        f.write((tab * l[0]) + l[1] + "\n")

def construct_bash_script(launch_conf: launch_configuration):
    bash_name = "simple_run_commands.sh"

    f = open("./%s"  % (bash_name), "w")

    #Set file to be executable by all
    os.system("chmod a+x ./%s" % (bash_name))
    
    #bash needs this becasuse ???
    f.write("#!/bin/bash\n\n\n")
    
    #get rid of previous build relics
    f.write("rm -r build install\n\n")

    #collect all packages in "packages_to_launch" and add them to colcon build
    #apparently you can do list comprehension on string formatting. neat

    #this breaks every package down into seperate strings, adds a space before each package, and then combines them into one new string to append to the bash script
    pkgs_to_build = []
    for pkg in launch_conf.packages_to_run:
        if pkg.build == True:
            pkgs_to_build.append(pkg)

    f.write("colcon build --packages-select" +  \
        "".join( [" %s" % pkg.name for pkg in list(pkgs_to_build)]) + "\n\n" )

    #Source ros2, and the newly built package
    f.write("source /opt/ros/foxy/setup.bash\n\n")
    f.write("source install/setup.sh\n\n")

    #launch your package's launch file.
    f.write("ros2 launch %s  %s" % (launch_conf.config_store_pkg.name, launch_conf.launch_file))
    
    #launch the launch file with your desired packages inside

    #close the write stream to make it unoccupied and launch the bash script as a sub process which pipes back to this temrinal
    f.close()
    rc = subprocess.call("./%s" % bash_name)

def create_urdf_of_model(launch_conf: launch_configuration, macro_folder_location: str, FCStd_name: str, urdf_name: str):
    """
    Take a FreeCAD model, and convert it to a URDF, and place .DAE files of the model inside the package model directory of the rviz config's parent package:

    I.E:
        If rviz_config.conf is stored in model_pkg, then auto_urdf.urdf will be stored in model_pkg<the root one>/models
    """
    #)
    urdf_config_path = "%s/urdf_file_settings.json"  % macro_folder_location
    f = open(urdf_config_path, "w")
    #f.write("")
    params = {
        "project_dir": os.getcwd(),
        "pkg": launch_conf.config_store_pkg.name,
        "FCStd": FCStd_name,
        "urdf_name": urdf_name,
    }
    f.write(json.dumps(params))
    f.close()
    os.system("python3 freecad_macros/export_model_to_urdf.py")

env_to_use = real_rviz_env_conf
"""ros2 configuration to use, look at lauch configurations to see what each one does."""

def launch_gazebo_world(launch_conf: launch_configuration):
    """generate an urdf file, convert that to an sdf file, then launch that sdf file."""
    create_urdf_of_model(env_to_use, freecad_macros_folder_name, model_file_name, launch_conf.urdf_file)
    sdf_path =  "%s%s.sdf" % (launch_conf.config_store_pkg.config.urdf_folder_path)
    os.system("gazebo.gz sdf -p %s > %s" % (launch_conf.urdf_file, sdf_path))
#os.system("python3 freecad_macros/export_model_to_urdf.py")
#os.system("gazebo.gz service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req 'sdf_filename: \"")

#construct_bash_script(env_to_use)
#launch_gazebo_world(env_to_use)
construct_bash_script(env_to_use)

