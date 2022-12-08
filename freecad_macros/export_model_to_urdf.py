#FreeCad wiki:


import os
import sys
#import collada
#print("collada is located at %s " % collada.__file__)
#print(os.environ['HOME'])
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
models_folder = "freecad_macros/"
external_libs_dir = os.environ['HOME']

"""
FREECAD WIKI FOR COMMANDS
https://wiki.freecadweb.org/Debugging#Python_Debugging
"""

if(FreeCAD == ""):
    import collada
    collada_path = collada.__file__
    print("STARTUP: script probably not executed as macro inside FreeCAD. re-running script through freecad")

    #runs Freecad via cmd in cmd mode and appends this macro and grabs 
    os.system("freecad -c ./freecad_macros/%s %s" % (macro_name, collada_path))
    exit()
    


class Model():
    """
    Save everything relevant about them model here, and document any particular parameters here as well since FreeCAD has no IDE integration :<
    
    NOTES:
    ---
        1: FreeCAD's internal model params should NOT be saved as self. Its data clutter. Save relevant individual parts of the model like position to self though.
    """
    def __init__(self, model_doc: str, model_name: str, *sub_models, color="white"):
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
            This is the model as it exists in FreeCAD
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
        self.center_xyz = [self.model.Shape.CenterOfMass[0], self.model.Shape.CenterOfMass[1], self.model.Shape.CenterOfMass[2]]
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
        self.origin = [self.model.Placement.Base[0], self.model.Placement.Base[1], self.model.Placement.Base[2]]
        """
        origin:
        ---
            start point of the sketch plane of the model. This CAN be different then centroid and is required to make position math relative to center of the object.
        """

    def __repr__(self):
        print("%s SPECS:" % self.model_name)
        print("=============")

        print("model center: %s " % self.center_xyz)
        print("model parents: %s" % self.parents)
        print("model origin in FreeCAD: %s" % self.origin)
        return ""


    def define_self_as_urdf(self):
        
        urdf_dir = models_folder + "freecad_urdf.xml"
        self_as_urdf = []

        self_as_urdf.append([0 ,"<link name=\"%s_link\">" % self.model_name])
        \
            self_as_urdf.append([1, "<visual>"]) #VISUAL
        \
                self_as_urdf.append([2, "<origin xyz=\"%s %s %s\">" % (self.center_xyz[0], self.center_xyz[1], self.center_xyz[2])])   
        \
                self_as_urdf.append([2, "<geometry>"]) #GEOMETRY
        \
                    self_as_urdf.append([3 ,"<mesh filename=\"%s\"/>" % urdf_dir])
        \
                self_as_urdf.append([2, "</geometry>"])
        \
        """
                self_as_urdf.append([3, "<material>"]) #MATERIAL 
        \
                    #self_as_urdf.append([4, "<color rgba=\"%s\"/>" % self.color])
        \
                self_as_urdf.append([3, "</material>"])
        
        """
        \
            self_as_urdf.append([1, "</visual>"])
        \
        self_as_urdf.append([0, "</link>"])

        f = open(urdf_dir, "w")
        for l in self_as_urdf:
            f.write("    " * l[0] + l[1] + "\n")


    def export_self_as_dae(self):
        """
        THIS FUNCTION IS NOT READY YET. WAIT FOR BUG FIX IN:
        https://github.com/FreeCAD/FreeCAD/issues/7989

        export self as a .dae model for rviz2

        NOTE
        ---
            IF PYCOLLADA IS NOT FOUND, THEN FOLLOW GUIDE TO FIX HERE
                TODO: 
                    MAKE THIS FIX AUTOMATIC BY DOWNLOADING FROM GIT AND PUTTING FILE IN BIN AUTOMATICALLY.
            https://wiki.freecadweb.org/Extra_python_modules#pyCollada

            https://forum.freecadweb.org/viewtopic.php?f=4&t=41949&start=10
        """
        #dae_folder = "./%s%s.dae" % (models_folder,self.model_name)
        #print(sys.executable[0:5])
        if(sys.executable[0:5] == "/snap"):
            print("FreeCAD installation is a snap. Adding ~/home installation of pycollada to PATHS to bypass pycollada not being in snap(????)")
            #See -> https://forum.freecadweb.org/viewtopic.php?t=17978
            sys.path.append(0, sys.argv[sys.argv.__len__() - 1])
            self.expose_paths()

            dae_folder = "/home/rydb/projects/group_robot_ros2/freecad_macros/%s.dae" % self.model_name
            print(dae_folder)
            import collada
            
            print(collada.__file__)
        #import importDAE
        #importDAE.export(self.model, dae_folder)
        
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


#going to want to unhard code this later
robot_model_dir = "/home/rydb/projects/group_robot_ros2/src/model_pkg/models/urdfmodel.FCStd"
wheel = Model(robot_model_dir, "LeftWheel")

#print("test arg is %s " % sys.argv[sys.argv.__len__() - 1])
#wheel.export_self_as_dae()
wheel.define_self_as_urdf()

#wheel.expose_paths()
#print(wheel)
exit()
#print(wheel)