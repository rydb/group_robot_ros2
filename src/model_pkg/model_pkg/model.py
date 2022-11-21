#For importing time
import rclpy
import tf2_msgs
#tf2 stuff,
import tf2_ros
#Need to get /Twist messages from the /cmd_vel topic from encoderless_teleop keyboard
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.clock import ROSClock
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float32, Int32MultiArray, String
import os
import glob

import numpy

numpy.linalg.eigvals

class Model():
    """
    A model consists of a base(non moving parts) and wheels(the moving parts)

    these are split up into /base_link and /wheel_link 1 and 2 respectively. 
    """
    def __init__(self, stl_base, stl_wheels):
        
        #GEOMETRY
        self.base = stl_base
        self.center_of_mass = None

        self.wheels = stl_wheels
    
    def create_base_link_from_stl(self, stl_b):
        """
        create the /base_link to publish from the stl_base
        
        a base link has:
            /
        """
        

    def create_wheel_links_from_stl():
        """
        create /wheel_link(s) to publish from the stl_wheels
        """


def publish_model():
    """
    collects all of the stls that the model uses, creates a topic, populates it with relevant information, then publishes it. to ros2.
    """
    

    pass

def main():
    package_name = "model_pkg"
    print("the current file being executed is %s" % os.path.dirname(os.path.realpath(__file__)))
    #print(os.path.join('share/%s/launch') % "model_pkg", glob.glob('launch/*.py'))
    #print(os.path.join('share', package_name, 'launch'),
    #    glob.glob('launch/*.py'))
    #print('Hi from model_pkg.')

if __name__ == '__main__':
    main()
