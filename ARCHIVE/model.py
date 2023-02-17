"""
file to act as an intemediary between setting what model the user the model wants, and how that model would exist in a given urdf converter.

E.G: For a FreeCAD model -> urdf converter, it would 


"""

from dataclasses import dataclass
from typing import *
from .materials import *
import yaml
import typing

@dataclass
class generic_model():
    """generic ros2 model to be interpreted by FreeCAD. Could be used for a different ros2 converter if you had a urdf convert for that."""
    package_dir: str
    """
    path to the ROS2 package which this model originates from
    
    I.E: if this model is in model_pkg/models/, then this should be the path to the root folder of model_pkg
    """
    model_path: str
    """path to the file which contains this model"""
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
    sub_models: Optional[List["generic_model"]] = None
    """sub models that are part of this model of this model"""

    #def as_dict(self):
    #    """return `!!!named after its ros_link-Name!!!` """
        
        #with open("%s%s.yaml" % self.package_dir, "w") as file:
    
            #yaml.dump(relevant_info_on_self, file)

    def return_as_dict_for_yaml(self):
        """take a list of models, recursively and return them as dict"""
        
        model_dict = {}
        if self.sub_models != None:
            for sub_model in self.sub_models:
                model_dict[sub_model.ros_link_name] = sub_model.return_as_dict_for_yaml()

            return model_dict
        else:
            return self.__dict__
                
#def load_model_from_yaml(label):
#    """loads a model from a yaml file based on its label(FreeCAD)/name"""

