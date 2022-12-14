#FreeCad wiki:


import os
import sys
FreeCAD = ""
try:
    import FreeCAD
    #import importDAE

except:
    pass

#list where packages are installed:
# https://stackoverflow.com/questions/122327/how-do-i-find-the-location-of-my-python-site-packages-directory

print("current python interpreter is: %s" % sys.executable)
#print(os.environ['HOME'])
# setup procedure if inside vscode or inside freecad:

current_env = sys.executable
macro_name = os.path.basename(__file__)
current_py_file = os.path.basename(__file__)


"""
FREECAD WIKI FOR COMMANDS
https://wiki.freecadweb.org/Debugging#Python_Debugging
"""

if(FreeCAD == ""):
    print("STARTUP: script probably not executed as macro inside FreeCAD. re-running script through freecad")

    #runs Freecad via cmd in cmd mode and appends this macro and grabs every sys.argv after first 3 commands and adds that to command
    final_console_cmd = "freecad -c ./freecad_macros/" + current_py_file
    print(sys.argv.__len__())
    for i in range(1, sys.argv.__len__()):
        print("APPENDED sys arg " + sys.argv[i])
        final_console_cmd += " " + sys.argv[i]
    print(final_console_cmd)
    os.system(final_console_cmd)
    exit()
    


class Model():
    """
    Save everything relevant about them model here, and document any particular parameters here as well since FreeCAD has no IDE integration :<
    
    NOTES:
    ---
        1: FreeCAD's internal model params should NOT be saved as self. Its data clutter. Save relevant individual parts of the model like position to self though.
    """
    def __init__(self, package_dir: str, model_doc: str, model_name: str, joint_type: str, ros_link_name: str, sub_models=None, color="white"):
        mm_to_m = 0.001
        """
        multiply by this num to get meters version of FreeCAD's default mm measurements.
        All measurements are stored in mm in FreeCAD(at least in 0.20.1?) in mm regardless of current set measurement unit.
        """

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
        self.model_doc = FreeCAD.open(model_doc)
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
        """
        model_name
        ---
            Name of model's Label in FreeCAD.
        """
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

        self.center_xyz = [self.model.Shape.CenterOfMass[0] * mm_to_m, self.model.Shape.CenterOfMass[1] * mm_to_m, self.model.Shape.CenterOfMass[2] * mm_to_m]
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
        self.origin = [self.model.Placement.Base[0] * mm_to_m, self.model.Placement.Base[1] * mm_to_m, self.model.Placement.Base[2] * mm_to_m]
        """
        origin:
        ---
            start point of the sketch plane of the model. This CAN be different then centroid and is required to make position math relative to center of the object.
        """
        #export self as dae since urdf needs these models

    def __repr__(self):
        print("%s SPECS:" % self.model_name)
        print("=============")

        print("model center: %s " % self.center_xyz)
        print("model parents: %s" % self.parents)
        print("model origin in FreeCAD: %s" % self.origin)
        return ""

    #def convert_to_m(mm_measure):
    #    """
    #   convert FreeCAD's default mm measurements to m.
    #    Model still exports models to mm regardless of measurement units from tesitng, so this assumes all models are measured in mm
    #    """
    #    return mm_measure * 0.001

    def get_self_as_urdf(self):
        """
        gets relevant information about self and return a list in urdf format. This is not exporting self as urdf, that uses this to get qualites of models for exporting.
        """
        l_list = []

        l_list.append([1 ,"<link name=\"%s_link\">" % self.ros_name])
        \
            l_list.append([2, "<visual>"]) #VISUAL
        \
                l_list.append([3, "<origin xyz=\"%s %s %s\"/>" % (self.center_xyz[0], self.center_xyz[1], self.center_xyz[2])])   
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

    def export_self_as_urdf(self):
        """"
        takes model and its children and converts it to an xml like list, and then transcribes it to "freecad_urdf.xml"

        REFERENCES:
        #Sample Turtlebot3 urdf
        https://github.com/ROBOTIS-GIT/turtlebot3/blob/foxy-devel/turtlebot3_description/urdf/turtlebot3_burger.urdf

        #guide for creating a urdf
        https://docs.ros.org/en/galactic/Tutorials/Intermediate/URDF/URDF-Main.html
        """
        #export as dae since urdf needs to read dae file
        self.export_self_as_dae()

        urdf_dir = self.urdf_folder + sys.argv[-1]
        print("URDF_DIR IS %s" % urdf_dir)
        self_as_urdf = []
        self_as_urdf.append([0, '<robot name="diff_bot">'])
        
        #append this models link to the urdf
        for l in self.get_self_as_urdf():
            self_as_urdf.append(l)

        self_as_urdf.append(([0, '']))
        #append all sub model joints and links to the urdf
        for sub_model in self.sub_models:
            #export sub model as dae since urdf needs to read dae file
            sub_model.export_self_as_dae()
            self_as_urdf.append([1, '<joint name="%s_joint" type="%s">' % (sub_model.ros_name, sub_model.joint_type)])
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

    def export_self_as_dae(self):
        """
        export self as a .dae model for rviz2
        """
        
        dae_folder = "%s%s.dae" % (self.models_folder,self.ros_name)

        try:
            import importDAE
        except:
            print("WARNING: collada likely not installed, attempting fix using freecad.pip util that comes with snap install assuming user installed FreeCAD via snap(where error normally occurs)")
            os.system("freecad.pip install pycollada")
            import importDAE

        importDAE.export(self.model, dae_folder)
        
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

model_pkg_dir = "%s/src/%s" % (sys.argv[-4], sys.argv[-3]) #"/home/rydb/Projects/group_robot_ros2/src/model_pkg"
robot_model_dir = "%s/models/%s" % (model_pkg_dir, sys.argv[-2]) #/home/rydb/Projects/group_robot_ros2/src/model_pkg/models/urdfmodel.FCStd"


wheel_left = Model(model_pkg_dir, robot_model_dir, "LeftWheel", "continuous", "left_wheel")
wheel_right = Model(model_pkg_dir, robot_model_dir, "RightWheel", "continuous", "right_wheel")
body = Model(model_pkg_dir, robot_model_dir, "BodyBase", "fixed", "base", sub_models=[wheel_left, wheel_right])

body.export_self_as_urdf()
#print("this runs successfully! %s" % sys.argv)

exit()
