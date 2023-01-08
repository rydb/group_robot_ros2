#FreeCad wiki:

import os
from dataclasses import dataclass
import sys
import numpy as np
import json


FreeCAD = ""
try:
    import FreeCAD
    #import importDAE

except:
    
    pass

#DEPENDENCIES FOR FREECAD MACRO
try:
            import trimesh
            #import pyglet
except:
    os.system("freecad.pip install trimesh")
    import trimesh


#list where packages are installed:
# https://stackoverflow.com/questions/122327/how-do-i-find-the-location-of-my-python-site-packages-directory

print("current python interpreter is: %s" % sys.executable)
#print(os.environ['HOME'])
# setup procedure if inside vscode or inside freecad:

current_env = sys.executable
macro_name = os.path.basename(__file__)
current_py_file = os.path.basename(__file__)


FreeCAD_mode_options = {
    "cmd": " -c ", # cmd mode is for good for fetching vars from outside scripts.
    "gui": " ", # gui mode is good for setting up enviorment vars in FreeCAD env without having to manually retype them
}
"""
dict that determines which holds which mode to launch FreeCAD in 
0 = console mode
1 = gui mode(crashes when ran. FreeCAD Does not give Verbose errors :/)
"""
FreeCAD_mode = "cmd"

"""
FREECAD WIKI FOR COMMANDS
https://wiki.freecadweb.org/Debugging#Python_Debugging
"""

if(FreeCAD == ""):
    print("STARTUP: script probably not executed as macro inside FreeCAD. re-running script through freecad")

    #runs Freecad via cmd in cmd mode and appends this macro and grabs every sys.argv after first 3 commands and adds that to command
    final_console_cmd = "freecad %s ./freecad_macros/" % FreeCAD_mode_options[FreeCAD_mode] + current_py_file
    os.system(final_console_cmd)

    #exit to prevent errors from FreeCAD not being defined
    exit()
    
@dataclass
class Material():
    """
    Represents material properties.
    
    NOTE: FreeCAD does not appear to have a innate way to give models material properties, so materials will need to be defined here.

    See: https://wiki.freecadweb.org/Material
    """
    name: str
    """name of material"""
    density: float
    """
    density of material in grams per cubic centimeter(g/cc)/(g/cm^3)
    
    get density of material from here:
    https://www.matweb.com/
    """



        
Generic_PETG = Material("PETG", 1.319)
"""'Generic' PETG. Should be the average of PETG properties"""

class Model():
    """
    Save everything relevant about them model here, and document any particular parameters here as well since FreeCAD has no IDE integration :<
    
    NOTES:
    ---
        1: FreeCAD's internal model params should NOT be saved as self. Its data clutter. Save relevant individual parts of the model like position to self though.
    """
    def __init__(self, package_dir: str, model_doc_dir: str, model_name: str, joint_type: str, ros_link_name: str, material: Material, sub_models=None, color="white"):
        self.models_folder = "%s/models/" % package_dir 
        self.urdf_folder = "%s/urdf/" % package_dir


        self.package_dir = package_dir
        """
        package_dir
        ---
            The root package folder which stores/will store model info/urdf info.
            E.G:
                if model_pkg is the package where model information will be stored
                then `(project_directory)/model_pkg` is package_dir
                
                models will be stored in:
                `package_dir/models`

                the created urdf file will be stored in
                `package_dir/urdf`
        """
        self.package_name = package_dir.split("/")[-1]
        
        self.model_doc_dir = model_doc_dir
        self.model_doc_name = model_doc_dir.split("/")[-1].replace(".FCStd", "")
        #print("||DEFINING DOCUMENT|| opening document file: |%s|" % model_doc_dir )
        self.model_doc = FreeCAD.open(model_doc_dir)
        """
        model_doc
        ---
            'Document' object inside FreeCAD which holds models. 
        """
        
        self.model = self.model_doc.findObjects(Label=model_name)[0]
        """
        model
        ---
            This is the model as it exists in FreeCAD.

            when FreeCad does 'findObjects', or when refering to model variables stored in FreeCAD, this is what to reference.
        """
        if(self.model_doc.findObjects(Label=model_name).__len__() > 1):
            raise "More then 1 model found with the label %s, throwing error to prevent erros down the line" % model_name
        


        self.model_name = self.model.Label
        """Name of model's Label in FreeCAD."""
        self.material = material
        """material of model"""
        self.joint_type = joint_type
        """
        joint type in urdf E.G:
            Continous = revolves around an origin, E.G: wheel

            swivel = ???
        """
        self.ros_name = ros_link_name
        """
        ros_name:
        ---
            This is the model's name to be set inside the urdf file for links/joints and to be searched for when looking for the model in the ros2 topic list.

            to prevent ros2 simulation code from breaking because the model name changed in FreeCAD, the model's name and its ros2 link are set seperately
            E.G: if the model's name is `BaseBody`, you would want to instead input `base` for this
        """
        self.sub_models = sub_models
        """
        all sub models of this model. 
        """

        self.center_xyz = self.unit_to_m([self.model.Shape.CenterOfMass[0], self.model.Shape.CenterOfMass[1], self.model.Shape.CenterOfMass[2]])
        """
        center/centroid of the model:
        ---
            SHOULD be center of mass assuming the object is of a uniform material(it will be in simulation.... probably...)
        """
        self.parents = self.model.Parents
        """
        parents:
        ---
            according to freecad, this is all parent objects which hold the current model in the deisgn
        """
        self.origin = self.unit_to_m([self.model.Placement.Base[0], self.model.Placement.Base[1], self.model.Placement.Base[2]])
        """
        origin:
        ---
            start point of the sketch plane of the model. This CAN be different then centroid and is required to make position math relative to center of the object.
        """
        if(type(self.model) == Part.Feature):
            """
            If the object is a "Part.Feature", then the model is most likely a mirrored object.

            FreeCAD does not give you the placement of a mirrored object, but you can get it from getting the "normal"/perpendicular line of the plane, and the
            position of the mirror'd object's "source"/parent object and multiplying by the opposite of the normal.

            this will give you the position of the mirrored object. 
            """

            round_n = 6

            flipped_normal = [round(self.model.Normal[0], round_n), round(self.model.Normal[1], round_n), round(self.model.Normal[2], round_n)]
            for i in range(0, flipped_normal.__len__()):
                if float(abs(flipped_normal[i])) == 0.0:
                    flipped_normal[i] = 1
                else:
                    flipped_normal[i] = flipped_normal[i] * -1
            
            #normal is a floating point array is extreme rounding errors, fix with above.

            self.origin = self.unit_to_m([self.model.Source.Placement.Base[0] * flipped_normal[0]\
                , self.model.Source.Placement.Base[1] * flipped_normal[1]\
                    , self.model.Source.Placement.Base[2] * flipped_normal[2]])


        self.mesh_location = self.export_self_as_dae()
        """export mesh to dae and return its location"""
        self.mesh: trimesh.Trimesh = trimesh.load(self.mesh_location, force="mesh")
        self.mesh.density = self.material.density



    def unit_to_m(self, messure_l: list, meassure_type = "mm"):
        """
        Take all measurements in an array from `mm` and convert to `m`(by default)

        Defaults to millitmeters since thats FreeCAD default unit for lengths.
        
        Add your measurement to measure_to_m if it isn't in there.
        """
        round_n = 6 
        measure_to_m = {
            "mm": 0.001,
        }
        multiplier = measure_to_m[meassure_type]
        return [round(x * multiplier, round_n) for x in messure_l]

    def __repr__(self):
        print("%s SPECS:" % self.model_name)
        print("=============")

        print("model center: %s " % self.center_xyz)
        print("model parents: %s" % self.parents)
        print("model origin in FreeCAD: %s" % self.origin)
        print("model document directory is: %s" % self.model_doc_dir)
        return ""


    def export_self_as_dae(self):
        """
        export self as a .dae model for rviz2

        also return location of exported file
        """
        origin = FreeCAD.Vector(0, 0 , 0)
        print("||DAE EXPORT|| exporting %s" % self.model_name)
        dae_folder = "%s%s.dae" % (self.models_folder,self.ros_name)
        #documentation of importDAE
        # https://github.com/FreeCAD/FreeCAD/blob/d35400aae339d71d5fc8c7f767e3be17687fae90/src/Mod/Arch/importDAE.py
        try:
            import importDAE
        except:
            print("WARNING: collada likely not installed, attempting fix using freecad.pip util that comes with snap install assuming user installed FreeCAD via snap(where error normally occurs)")
            os.system("freecad.pip install pycollada")
            import importDAE

        __obj__ = self.model

        if(type(__obj__) == Part.Feature):
            print("||DAE EXPORT|| obj is feature, assuming this is a mirrored feature untill more part features are added to this")
            #placement is not in placement for Part.Features, but in .Base?
            __obj__.Base = origin
        #Set the object's placement to be at world origin since models are not exported relative to their origin, but to world origin.
        
        #by setting the object's position to world origin, the model will be exported relative to it's origin.
        else:    
            __obj__.Placement.Base = FreeCAD.Vector(0, 0, 0)

        print("||DAE EXPORT|| finished DAE export")
        importDAE.export(__obj__, dae_folder)

        return dae_folder

    def symetric_inertia_tensor(self):
        """
        get "symetric" inertia tensor of model in SI units.
        
        The SI units being either kg.m^2? or g.cm^2?(gazebo doesn't say which??)

        "symetric" meaning only 6 inertia values,
        `ixx, ixy, ixz,
        iyy, iyz, izz`

        are nessecary, and not the full inertial tensor.

        See: https://classic.gazebosim.org/tutorials?tut=inertia
        """
        #helpful resource: https://towardsdatascience.com/how-to-voxelize-meshes-and-point-clouds-in-python-ca94d403f81    

        #voxelized_mesh: trimesh.voxel.VoxelGrid = mesh.voxelized(0.001)
        inertia = self.mesh.mass_properties["inertia"]
        return [inertia[0][0], inertia[0][1], inertia[0][2], inertia[1][1], inertia[1][2], inertia[2][0]]
        

    def get_self_as_urdf(self):
        """
        gets relevant information about self and return a list in urdf format. This is not exporting self as urdf, that uses this to get qualites of models for exporting.
        """
        l_list = []

        l_list.append([1 ,"<link name=\"%s_link\">" % self.ros_name])
        \
            l_list.append([2, "<inertial>"])
        \
                sym_inertia = self.symetric_inertia_tensor()
        \
                l_list.append([3, '<mass value="%s"/>' % self.mesh.mass])
        \
                l_list.append([3, '<inertia ixx="%s" ixy="%s" ixz="%s" iyy="%s" iyz="%s" izz="%s"/>' % (\
                    sym_inertia[0], sym_inertia[1], sym_inertia[2], sym_inertia[3], sym_inertia[4], sym_inertia[5])])
                    #ensure inertia tensor "satisfies the triangle inequality, ie. ixx + iyy >= izz, ixx + izz >= iyy and iyy + izz >= ixx."
                    #https://physics.stackexchange.com/questions/48266/can-any-physical-rigid-body-be-represented-by-an-ellipsoid-with-the-same-angular/48273#48273
                    #https://classic.gazebosim.org/tutorials?tut=inertia    
        \
            l_list.append([2, "</inertial>"])
        \
            l_list.append([2, "<visual>"]) #VISUAL
        \
                if(self.sub_models == None): # DO NOT SET ORIGIN FOR BASE MODEL SINCE THE ORIGIN FOR THAT SHOULD STAY WHERE IT IS
                    pass
                    #l_list.append([3, "<origin xyz=\"%s %s %s\"/>" % (self.center_xyz[0], self.center_xyz[1], self.center_xyz[2])])   
        \
                l_list.append([3, "<geometry>"]) #GEOMETRY
        \
                    l_list.append([4 ,"<mesh filename=\"package://%s/models/%s.dae\"/>" % (self.package_name, self.ros_name)])
        \
                l_list.append([3, "</geometry>"])
        \
        """
                self_as_urdf.append([3, "<material>"]) #MATERIAL 
        \
                    #self_as_urdf.append([4, "<color rgba=\"%s\"/>" % self.color])
        \
                self_as_urdf.append([3, "</material>"])
        
        """
        \
            l_list.append([2, "</visual>"])
        \
        l_list.append([1, "</link>"])

        return l_list

    def export_self_as_urdf(self, urdf_name="diff_bot.urdf.xml"):
        """"
        takes model and its children and converts it to an xml like list, and then transcribes it to file named after urdf_name

        REFERENCES:
        #Sample Turtlebot3 urdf
        https://github.com/ROBOTIS-GIT/turtlebot3/blob/foxy-devel/turtlebot3_description/urdf/turtlebot3_burger.urdf

        #guide for creating a urdf
        https://docs.ros.org/en/galactic/Tutorials/Intermediate/URDF/URDF-Main.html
        """
        print("||URDF EXPORT|| exporting self as urdf..")
        #export as dae since urdf needs to read dae file
        urdf_dir = self.urdf_folder + urdf_name
        #print("URDF_DIR IS %s" % urdf_dir)
        self_as_urdf = []
        self_as_urdf.append([0, '<robot name="diff_bot">'])
        
        #append this models link to the urdf
        for l in self.get_self_as_urdf():
            self_as_urdf.append(l)

        self_as_urdf.append(([0, '']))
        #append all sub model joints and links to the urdf
        if(self.sub_models != None):
            for sub_model in self.sub_models:
                #export sub model as dae since urdf needs to read dae file
                self_as_urdf.append([1, '<joint name="%s_joint" type="%s">' % (sub_model.ros_name, sub_model.joint_type)])
                \
                    self_as_urdf.append([2, "<origin xyz=\"%s %s %s\"/>" % (sub_model.origin[0], sub_model.origin[1], sub_model.origin[2])])
                \
                    self_as_urdf.append([2, '<parent link="%s_link"/>' % self.ros_name])
                \
                    self_as_urdf.append([2, '<child link="%s_link"/>' % sub_model.ros_name])
                \
                self_as_urdf.append([1, '</joint>'])
                self_as_urdf.append(([0, '']))
                for l in sub_model.get_self_as_urdf():
                    self_as_urdf.append(l)
                #whitespace
                self_as_urdf.append([0, ""])
            
        \
        self_as_urdf.append([0, '</robot>'])
        
        f = open(urdf_dir, "w")
        for l in self_as_urdf:
            f.write("    " * l[0] + l[1] + "\n")

        print("||URDF EXPORT|| finished exporting self ad urdf")


        #voxelized_mesh.export(self.ros_name + "_voxelized", file_type="binvox")
    def expose_paths(self):
        """
        Lists helpful debug info related to directories of things:
            1: Lists current python interpreter

            2:print all PATHs visible to the currently executing python file
        """
        print("CURRENT INTEPRETER IS: %s " % sys.executable)
        print("CURRENT PATHS ARE: ")
        print("===================")
        for p in sys.path:
            
            print(p)
    def export_self_as_GUI_commands(self):
        """
        Picking apart the API for commands requries a macro or entering commands.
        define self as a model so in a running FreeCAD instance, commands to pick apart the api can be done faster.
        """
        doc_name = "doc"
        cmd_l = []
        
        #define document
        cmd_l.append("%s = FreeCAD.getDocument('%s')" % (doc_name, self.model_doc_name))

        #define model
        cmd_l.append("%s = %s.findObjects(Label='%s')[0] " % (self.model_name, doc_name, self.model_name))

        for l in cmd_l:
            print(l)
         

        
#get project configs from urdf_file_settings.json
try:
    f = open("%s/urdf_file_settings.json" % os.path.dirname(__file__), "r")
except Exception as e:
    print("urdf_file_settings.json hasn't been initialized yet. Run simple run's urdf generator once and it will generate it")
    print(os.getcwd())
    raise Exception("Actual Exception: " + str(e))


config = json.load(f)

project_dir = config["project_dir"]
pkg = config["pkg"]
FCStd = config["FCStd"]
urdf_name = config["urdf_name"]
#"""
#project_dir of hard coded directories. this is not used when this macro is being called from simple_run.py
#"""

model_pkg_dir = "%s/src/model_pkg" % project_dir
robot_model_dir = "%s/src/model_pkg/models/urdfmodel.FCStd" % project_dir
urdf_dir = "%s/src/model_pkg/urdf/" % project_dir


wheel_left = Model(model_pkg_dir, robot_model_dir, "LeftWheel", "continuous", "left_wheel", Generic_PETG)
wheel_right = Model(model_pkg_dir, robot_model_dir, "RightWheel", "continuous", "right_wheel", Generic_PETG)

#sub_models=[wheel_left, wheel_right]
body = Model(model_pkg_dir, robot_model_dir, "BodyBase", "fixed", "base", Generic_PETG)

#body.export_self_as_GUI_commands()
#wheel_left.export_self_as_GUI_commands()
#wheel_right.export_self_as_GUI_commands()
#wheel_right.export_self_as_dae()
print(body.export_self_as_urdf())
#print(body)
#body.export_self_as_urdf()

exit()
