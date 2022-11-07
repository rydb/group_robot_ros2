#tesitng simple code that doesn' needing building a colcon package for.

from difflib import diff_bytes
from multiprocessing.forkserver import read_signed
import numpy
import numpy as np
import os
import binascii
import time



model_dir = "models/"
model_name = "urdfmodel.stl"
"""
import stl
# Create a new plot
figure = pyplot.figure()
axes = mplot3d.Axes3D(figure)

# Load the STL files and add the vectors to the plot
your_mesh = stl.mesh.Mesh.from_file('models/urdfmodel.stl')
axes.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh.vectors))

# Auto scale to the mesh size
scale = your_mesh.points.flatten()
axes.auto_scale_xyz(scale, scale, scale)

# Show the plot to the screen
pyplot.show()
"""

class Model:



    def __init__(self, model_dir: str, model_name: str, model_unit_of_measurement: str, *sub_models):

        __measurement_multiplier = {
        "mm": 0.001
        }
        """
        multiplier value to multiply vectors by to conver them to meters for ROS2
        
        """

        self.model_dir = model_dir
        self.model_name = model_name

        self.len_to_m_mult = __measurement_multiplier[model_unit_of_measurement]
        self.length = None
        self.width = None
        self.height = None
        self.centroid = None


        self.read_obj_model()
        #collect everything relevant about the model like length, height, controid, etc, and set relevant model's
        #self details inside method.

        self.construct_urdf("diff_drive.urdf")

    #def __repr__(self):
        
        #pass

    def fetch_self_as_xacro(self):
        """
        return string list of properties about self like length, width, height, etc.. in xacro format
        so that they can be added to the final urdf .xacro file by the top-most parent model.
        """
        xacro_list = []

        xacro_list.append("xacro:property name=\"%s_length\" value=\"%s\"\n" % (self.model_name, self.length))
        xacro_list.append("xacro:property name=\"%s_width\" value=\"%s\"\n" % (self.model_name, self.width))
        xacro_list.append("xacro:property name=\"%s_height\" value=\"%s\"\n" % (self.model_name, self.height))

    def read_obj_model(self):
        """
        Reads .obj model/mesh and sets this model's attributes to match the collected data. 

        DOCS: 
        ---
            https://en.wikipedia.org/wiki/Wavefront_.obj_file

        """

        #bench mark how long it takes to do everything so I can speed this up for bigger .obj files
        time_at_start = time.time()
        file = open(self.model_dir)

        file_lines = file.readlines()



        #replace all \n instances
        file_lines = [l.replace("\n", "") for l in file_lines]

        #remove all comments, remove them after splitting lines by character so indentation next to "#" doesn't
        #accidently cause commented lines to be left for further parsing.

        for i in range(0, file_lines.__len__() - 1):
            if file_lines[i][0] == "#":
                file_lines.pop(i)


        split_categories = [v.split(" ") for v in file_lines]
        
        #save all vectors ass np.array
        #vectors = np.array()
        vectors = []
        for l in split_categories:
            if l[0] == "v":
                vectors.append(l[1:])

        #convert vectors to a numpy array after getting every vector into a python list.
        vectors = np.array(vectors).astype(float)
        #print(vectors[:, 0])


        
        #nbtriangles = int.from_bytes(fichier.read(4), 'little')

        #fetch length width and height of the model.

        #model x and y are swapped, but for code maintainability I wrote things in order of length width and height.
        #will need to rotate model later. 

        #get absolute values of highest and lowest x y z vectors, add them together and set length width height to
        #be equal to each one respectively
        self.length = (np.abs(np.amin(vectors[:, 0])) + np.abs(np.amax(vectors[:, 0]))) * self.len_to_m_mult
        self.width = (np.abs(np.amin(vectors[:, 1])) + np.abs(np.amax(vectors[:, 1])))  * self.len_to_m_mult
        self.height = (np.abs(np.amin(vectors[:, 2])) + np.abs(np.amax(vectors[:, 2])))  * self.len_to_m_mult

        self.centroid = np.array( [self.length/2, self.width/2, self.height/2] )



        print("lengh is %s, width is %s height %s" % (self.length, self.width, self.height))

        print("centroid is %s " % self.centroid)

        print("it took %s seconds to run all operations involving the obj file" % (time.time() - time_at_start))

    def construct_urdf(self, name_of_urdf):
        """
        take all model params and construct a urdf file for ROS2

        I can't believe people make xacros by hand...
        """
            

        tab = "    "
        
        f = open("./%s"  % (name_of_urdf), "w")

        f.write("<?xml version \"1.0\">\n")
        #set xml version.

        f.write("<robot name=\"diff_bot\" xmlns:xacro=\"http://ros.org/wiki/xacro\">\n")

        """
        PROPERTIES OF ROBOT
        """
        f.write(tab + "xacro:property name=\"%s_length\" value=\"%s\"\n" % (self.model_name, self.length))
        f.write(tab + "xacro:property name=\"%s_width\" value=\"%s\"\n" % (self.model_name, self.width))
        f.write(tab + "xacro:property name=\"%s_height\" value=\"%s\"\n" % (self.model_name, self.height))

        #f.write("xacro:include filename=\"")

        f.write("</robot>")
        #end of robot xml


def calculate_mass_of_stl():
    """
    get the mass of an stl
    """
wheel = Model("models/urdfmodel-Wheel.obj", "wheel", "mm")
base_body = Model("models/urdfmodel-BodyBase.obj", "base", "mm")