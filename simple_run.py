#Constructs a bash script to source, compile, remove relics, etc.. and run a user defined set of packages.

import os
import subprocess
import glob


class launch_configuration():
    """
    object that holds all of the parameters for a specific type of a ros2 enviorment. E.G: configuration for running a simulation enviorment/real enviorment.
    
    packages to build
    --
    should include every packagfe required to launch your launch_configuration

    launch_pkg_n_name
    --
    should include the package with your launch file, and the name of that launch file(with the .py extension)
    """
    def __init__(self, packages_to_build: dict, launch_pkg_n_name: str, pkg_n_rviz_config: list):
        self.packages_to_build = packages_to_build
        self.launch_pkg_n_name = launch_pkg_n_name
        self.pkg_n_rviz_config = pkg_n_rviz_config

sim_env_conf = launch_configuration(
    {
        "model_pkg": "model", #package name, name of python file being executed without.py extension(E.G: model.py = model)
    },
    ["model_pkg", "sim_launch.py"], #package_name, launch_file to launch in package_name/launch/
    ["model_pkg", "rviz_config_test.rviz"] #package_name, rviz_config file in package_name/rviz/
    )



"""
launch file in the root package(should be in the same folder as this script)
"""

"""
Packages for the bash script to compile and launch, I.E:

```
"publisher_pkg": "publisher"
```
"publisher_pkg" is the package
"publisher" is the name of the node
"""

def ammend_setup_py(launch_conf: launch_configuration):
    """
    adds EVERY folder in each package_to_build to their /share directorys for each setup.py

    ROS2 requires everything it uses to be in the /share directory, so for the sake of saving headaches, pre-emptively add EVERYTHING to it. 
    """
    
    tab = "    "

    for key in launch_conf.packages_to_build.keys():
        print(key)
        directory_of_file = "src/%s/" % key
        file_to_edit = "setup.py"
        print(".%s*/" % directory_of_file)
        setup_py = directory_of_file + file_to_edit
        print("re-writting ||%s|| to include all folders inside them inside their /install/<pkg_name>/share directory" % key)

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
    f.write("colcon build --packages-select" +  \
        "".join( [" %s" % pkg for pkg in list(launch_conf.packages_to_build.keys())]) + "\n\n" )

    #Source ros2, and the newly built package
    f.write("source /opt/ros/foxy/setup.bash\n\n")
    f.write("source install/setup.sh\n\n")

    #launch your package's launch file.
    f.write("ros2 launch %s  %s" % (launch_conf.launch_pkg_n_name[0], launch_conf.launch_pkg_n_name[1]))
    
    #launch the launch file with your desired packages inside

    #close the write stream to make it unoccupied and launch the bash script as a sub process which pipes back to this temrinal
    f.close()
    rc = subprocess.call("./%s" % bash_name)

#ammend_setup_py(sim_env_conf)
construct_bash_script(sim_env_conf)

#print(glob.glob("./src/model_pkg/*/"))

