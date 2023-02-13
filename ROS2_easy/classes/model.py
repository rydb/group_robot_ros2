"""
file which represents individual model in any given model file

FreeCAD cannot be currently be interfaced with like a library, so an intermediate class for sending data back and forth is nessecary.
"""

from dataclasses import dataclass
from typing import *
from .materials import *

@dataclass
class generic_model():
    """generic ros2 model to be interpreted by FreeCAD. Could be used for a different ros2 converter if you had a urdf convert for that."""
    package_dir: str
    
    robot_model_path: str

    label: str
    """label of model in FreeCAD. Click on the model inside freecad and look for this"""

    joint_type: str
    """
    type of joint that this robot exists as in ROS2. 
    
        known types:
            `fixed`:
                a non moving part(e.g: a hull)
            `continuous`:
                revolves around a joint(e.g: a wheel)
    """
    ros_link_name: str
    """name of model as it would exist as a ros2 link, e.g: `base`, for `base_link`"""

    material: Material
    """material of model. See materials for materials"""

    sub_models: Optional[List["Model"]] = None
    """sub models that are part of this model of this model"""