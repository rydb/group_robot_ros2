#tesitng simple code that doesn' needing building a colcon package for.

from difflib import diff_bytes
from multiprocessing.forkserver import read_signed
import numpy
import numpy as np
import os
from lxml import etree as et
import time
import zipfile
import re
import sympy as sym
import glob
#Help resources for figuring out urdfs/etc...
"""
xacro how to
https://bitbucket.org/theconstructcore/box_bot/src/galactic/

run freecad in ros2:
https://stackoverflow.com/questions/67325416/how-to-embed-freecad-in-a-python-virtual-environment

How to link external spreadsheets in freecad:
https://forum.freecadweb.org/viewtopic.php?t=61990

xml vs json(see comments):
https://insights.dice.com/2019/01/25/xml-vs-json-difference-developers/

how to use lxml library
https://stackabuse.com/introduction-to-the-python-lxml-library/

how to build a urdf file:
https://docs.ros.org/en/galactic/Tutorials/Intermediate/URDF/URDF-Main.html

urdf file github:
https://github.com/ros/urdf_tutorial/tree/ros2/

Learning ROS2:
https://www.reddit.com/r/ROS/comments/w2zcgw/best_way_to_learn_ros/
"""

_measurement_multiplier = {
"mm": 0.001
}

models_dir = "models/"

urdf_dir = ""

class Spreadsheet():
    """
    class for sifting through spreadsheets more easily. 

    Currently works with xml.
    """
    def __init__(self, spreadsheet_dir, spreadsheet_type):
        self.spreadsheet_dir = spreadsheet_dir
        self.spreadsheet_type = spreadsheet_type

        self.spreadsheet_dict = "???"
        """
        spreadsheet result from reading as dict
        """

    def read_FreeCAD_spreadsheet(self, name_of_sheet):
        """
        read a FreeCAD Document.xml and parse variables from specific spreadsheet

        Data should follow this format in the FreeCAD spreadsheet:

        ```
        CELL: A:        CELL B:

        variable_name   value

        variable_two    value
        ```
        """
        if(self.spreadsheet_type != "xml"):
            raise("FreeCAD spreadsheets are saved in .xml files, but this Spreadsheet instance is set to be a .%s file" % self.spreadsheet_type )
        
        freecad_file = zipfile.ZipFile(self.spreadsheet_dir)
        #l_files = os.listdir("./urdfmodel.FCStd")
        freecad_xml_bytes = freecad_file.read("Document.xml")
        #freecad_xml_str = str(freecad_xml_bytes, 'UTF-8')
        
        root = et.fromstring(freecad_xml_bytes)
        #help(root)
        #spreadsheet_xml = root.findall(".//Property[@name='cells']")[0].getchildren()[0].getchildren()
        spreadsheet_xml = root.find(".//Property[@name='cells']").find(".//Cells")
  
  
        """
        In order to get the spreadsheet of a model, you first must 
        """

        spreadsheet_list_A = []
        spreadsheet_list_B = []

        #Do an initial pass over the element object to collect all of the spreadsheet cells and put them into seperate
        #lists Cell A is the categories/parameters, Cell B is their value.
        for i in range(0, spreadsheet_xml.__len__()):


            if(spreadsheet_xml[i].tag == "Cell"):
                if(spreadsheet_xml[i].attrib["address"][0] == "A"):
                    spreadsheet_list_A.append(spreadsheet_xml[i].attrib)
                elif(spreadsheet_xml[i].attrib["address"][0] == "B"):
                    spreadsheet_list_B.append(spreadsheet_xml[i].attrib)
                else:
                    raise Exception("something changed under the hood about how FreeCAD handles spreadsheets, expected A or B labeled cell but got \"%s\" go to spreadsheet in Document.xml in FreeCadstd file to figure out what went wrong!" % spreadsheet_xml[i].attrib["address"]) 
    
        spreadsheet_dict = {}
        for i in range(0, spreadsheet_list_A.__len__()):
            cell_a = spreadsheet_list_A[i]["content"]
            
            cell_b = spreadsheet_list_B[i]["content"]

            cell_b_messur_unit = re.findall('\D[a-z]', cell_b[cell_b.__len__() - 3 : cell_b.__len__()])
            #print(cell_b)

            #check to see if a measurement unit was found, and weather that measure unit exists as a
            # _measurement_multiplier*
            #*in order to convert to it meters to be SI
            if((cell_b_messur_unit.__len__() > 0) and cell_b_messur_unit[0] in list(_measurement_multiplier.keys()))  :
                cell_b = int(re.sub('\D', '', cell_b)) * _measurement_multiplier[cell_b_messur_unit[0]]
            else:
                #cell_b.split
                for s in cell_b.split(" "):
                    try:
                        s = s.strip("=")
                        #print("s is %s, current dict is %s" % (s, spreadsheet_dict))
                        #print(cell_b)
                        cell_b = cell_b.replace(s, str(spreadsheet_dict[s])).strip("=")
                        #print(s)
                        #print(cell_b)
                    except KeyError as e:
                        pass
                        #print("got error what!")
                pass
            """
            gets the meassurement unit(mm/m, etc.....) and puts them into their own variables with their Regexp

            cell_b_messur_unit finds all instances of charcters without numbers for the last 3 characters of the string.
            The last 3 characters should be the measurement unit for the value, so should extract something like.
            "mm", or "cm", or "m"

            this may break with other measurement units other then mm, but im not sure..



            cell_b_num deletes everything that isnt a number.

            re.sub removes the characters that don't match those categories
            """
            spreadsheet_dict[spreadsheet_list_A[i]["content"]] = cell_b

        self.spreadsheet_dict = spreadsheet_dict

        #Do one final runthrough with sympy to get rid of math operators and turn all results into singular floats
        for key in self.spreadsheet_dict.keys():


            expres = str(self.spreadsheet_dict[key])

            #Clean up string of characters that could be used in part of a eval exploit
            #since sympy devs decided making .simplify use eval() was good idea...
            expres = ''.join(c for c in expres if c.isalpha() != True)
            expres = expres.strip("__")

            #convert sympy float back into python float.
            self.spreadsheet_dict[key] = round(float(sym.simplify(expres)), 5)


class Model:
    
    def __init__(self, model_dir: str, model_name: str, model_unit_of_measurement, *sub_models, color="white",):

        __measurement_multiplier = {
        "mm": 0.001
        }
        self.color_list = {
            "cyan": "0 1.0 1.0 1.0",
            "white": "1.0 1.0 1.0 1.0"
            
        }
        self.self_as_urdf = []
        """
        list of defined colors, if you get an error thrown with your color, add a new color in
        rgba format to this dict.
        """
        self.color = self.color_list[color]
        """
        multiplier value to multiply vectors by to conver them to meters for ROS2
        
        """

        self.model_dir = model_dir
        self.model_name = model_name
        self.sub_models = sub_models


        self.len_to_m_mult = __measurement_multiplier[model_unit_of_measurement]
        self.length = None
        self.width = None
        self.height = None
        self.centroid = None


        self.read_obj_model()
        self.define_self_as_xml()
        #collect everything relevant about the model like length, height, controid, etc, and set relevant model's
        #self details inside method.

        #self.construct_urdf("diff_drive.urdf")
        #self.xacro_properties_list = self.fetch_self_as_urdf()

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
        vectors = []
        for l in split_categories:
            if l[0] == "v":
                vectors.append(l[1:])

        #convert vectors to a numpy array after getting every vector into a python list.
        vectors = np.array(vectors).astype(float)

        #fetch length width and height of the model.

        #model x and y are swapped, but for code maintainability I wrote things in order of length width and height.
        #will need to rotate model later. 

        #get absolute values of highest and lowest x y z vectors, add them together and set length width height to
        #be equal to each one respectively
        self.length = (np.abs(np.amin(vectors[:, 0])) + np.abs(np.amax(vectors[:, 0]))) * self.len_to_m_mult
        self.width = (np.abs(np.amin(vectors[:, 1])) + np.abs(np.amax(vectors[:, 1])))  * self.len_to_m_mult
        self.height = (np.abs(np.amin(vectors[:, 2])) + np.abs(np.amax(vectors[:, 2])))  * self.len_to_m_mult

        self.centroid = np.array( [self.length/2, self.width/2, self.height/2] )



        #print("lengh is %s, width is %s height %s" % (self.length, self.width, self.height))

        #print("centroid is %s " % self.centroid)

        #print("it took %s seconds to run all operations involving the obj file" % (time.time() - time_at_start))

    def define_self_as_xml(self):
        
        self.self_as_urdf.append([1 ,"<link name=\"%s_link\">" % self.model_name])
        \
            self.self_as_urdf.append([2, "<visual>"]) #VISUAL
        \
                self.self_as_urdf.append([3, "<origin xyz=\"%s %s %s\">" % (self.centroid[0], self.centroid[1], self.centroid[2])])   
        \
                self.self_as_urdf.append([3, "<geometry>"]) #GEOMETRY
        \
                    self.self_as_urdf.append([4 ,"<mesh filename=\"%s\"/>" % self.model_dir])
        \
                self.self_as_urdf.append([3, "</geometry>"])
        \
                self.self_as_urdf.append([3, "<material>"]) #MATERIAL 
        \
                    self.self_as_urdf.append([4, "<color rgba=\"%s\"/>" % self.color])
        \
                self.self_as_urdf.append([3, "</material>"])
        \
            self.self_as_urdf.append([2, "</visual>"])
        \
        self.self_as_urdf.append([1, "</link>"])

        """
        v- I cant figure out how to get this to work, for the mean time, xml line endings will be hand coded 
        for i in range(0, reversed_list.__len__()):
            xml_line = reversed_list[i][1]
            #print(xml_line)
            if(xml_line[xml_line.__len__() - 2 : xml_line.__len__()]) != "/>":
                print("appending %s " % xml_line)
                link_list.append([reversed_list[i][0], reversed_list[i][1].replace("<", "</")])
                pass
            else:
                pass
                #link_list.append([reversed_list[i][0], reversed_list[i][1].replace("<", "</")])

        """ 

    def construct_urdf(self, name_of_urdf):
        """
        take all model params for this model and its sub models and construct a urdf file for ROS2
        """
        tab = "    "
        print(self.self_as_urdf)
        f = open("./%s%s"  % (self.model_dir ,name_of_urdf), "w")
        f.write("<?xml version=\"1.0\"?>" + "\n")
        f.write("<robot name=\"diff_bot\" xmlns:xacro=\"http://ros.org/wiki/xacro\">" + "\n")

        for child in self.sub_models:
            for l in child.self_as_urdf:
                f.write(tab * l[0] + l[1] + "\n")
        for l in self.self_as_urdf:
            f.write(tab * l[0] + l[1] + "\n")

        f.write("</robot>")

def calculate_mass_of_stl():
    """
    get the mass of an stl
    """

#wheel = Model("models/urdfmodel-Wheel.obj", "left_wheel", "mm")
#wheel = Model("models/urdfmodel-Wheel.obj", "right_wheel", "mm")
#base_body = Model("models/urdfmodel-BodyBase.obj", "base", "mm")

spreadsheet = Spreadsheet("./urdfmodel.FCStd", "xml")
spreadsheet.read_FreeCAD_spreadsheet("specs_spreadsheet")
print(spreadsheet.spreadsheet_dict)


#base_body.construct_urdf("testurdf.urdf")