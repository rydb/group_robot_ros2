from dataclasses import dataclass
import typing
from typing import Optional
import os
import numpy as np

#!!!RELATIVE IMPORTS MUST HAVE A DOT OR THEY THROW AN ERROR WHEN THE FILE IMPORTING THE IMPORTING FILE TRYS THE IMPORT THE IMPORTING FILE!!!
from .Config import Config
PROJECT_DIRECTORY = os.getcwd() + "/"


class Package():
    """A ros2 package and its relevant information."""
    global local_directory_for
    local_directory_for = {
        "URDFS": "urdf/",
        "MODELS": "models/",
        "RVIZ": "rviz/",
        "PACKAGES": "src/",
        "PROJECT_DIRECTORY": PROJECT_DIRECTORY
    }
    """people store files like models, urdfs, etc.. in different ways, refer to this schema for file paths on where to find file types"""
    
    def __init__(self, parent_pkg, name, executable_name, config= None, urdf_name= None, output= "screen", build= False, optional_launch_file_node_args = None):
        self.parent_pkg: "Package" = parent_pkg
        """string name of ros2 package that is parent of this package."""
        self.name: str = name
        """name of this package"""
        self.executable_name: str = executable_name
        """
        name of executable used by package. In a ros2 package, this is generally the name of the file executing code,
            E.G:
            in model_pkg:
                `model_pkg -> src -> model_pkg -> model.py`
                is package executable, so

                `model` is executable_name
        """
        self.config: Optional[Config] = config
        """config info about this package if relevant"""

        self.urdf_name: str = urdf_name
        """name of urdf file + extension"""

        #self.urdf_path: str = self.urdf_folder + self.urdf_name

        output: Optional[str] = output
        """output for package, used in launch file"""

        self.build: Optional[bool] = build
        """weather colcon builds this package. is set to no by default"""

        self.optional_launch_file_node_args: Optional[typing.Dict[str, str]] = optional_launch_file_node_args

    def __repr__(self):
        return str(self.__dict__)

    @property
    def urdf_folder(self):
        """return urdf folder absolute path"""
        if(self.urdf_name !=  None):
            return "%s%s%s/%s" % (local_directory_for["PROJECT_DIRECTORY"],local_directory_for["PACKAGES"], self.name, local_directory_for["URDFS"])
        else:
            raise "NO URDF FILE HAS BEEN DECLARED FOR THIS PACKAGE, SET THE URDF'S FILE NAME ONE WHEN INITIALIZING THIS PACKAGE IF THIS PACKAGE IS MEANT TO HAVE ONE. THROWING ERORR TO PREVENT UNKNOWN BEHAVIOUR"
    
    @property
    def urdf_path(self):
        """return urdf file absoltue path"""
        if(self.urdf_name !=  None):
            return self.urdf_folder + self.urdf_name
        else:
            raise "NO URDF FILE HAS BEEN DECLARED FOR THIS PACKAGE, SET THE URDF'S FILE NAME ONE WHEN INITIALIZING THIS PACKAGE IF THIS PACKAGE IS MEANT TO HAVE ONE. THROWING ERORR TO PREVENT UNKNOWN BEHAVIOUR"


    

    """optional arguements to be a part of the launch file node list. Set this for packages with custom node arguements like rviz2."""
    def as_node_conf_dict(self):

        """launch files represent ros2 packages as dicitionaries. return this package as a dict for that"""
        result =  {
            "package": "'%s'" % self.name,
            "executable": "'%s'" % self.executable_name,
            "output": "'%s'" % self.output,
            "parameters": "[]"

        }
        if(self.optional_launch_file_node_args != None):
            result.update(self.optional_launch_file_node_args)
        return result


#model_pkg = Package(None, "model_pkg", "model", build=True, urdf_name="diff_bot.urdf.xml")

