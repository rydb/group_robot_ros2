#Constructs a bash script to source, compile, remove relics, etc.. and run a user defined set of packages.

import os
import subprocess

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
    def __init__(self, packages_to_build: dict, launch_pkg_n_name: str):
        self.packages_to_build = packages_to_build
        self.launch_pkg_n_name = launch_pkg_n_name

sim_env_conf = launch_configuration(
    {
        "model_pkg": "model",
    },
    ["model_pkg", "sim_launch.py"]
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

construct_bash_script(sim_env_conf)



