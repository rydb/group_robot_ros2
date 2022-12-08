from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
import ament_index_python
import os
import glob

import numpy

#numpy.linalg.eigvals

#Helpful resources:
# https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html

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



class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state
        tilt = 0.
        tinc = degree
        swivel = 0.
        angle = 0.
        height = 0.
        hinc = 0.005

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'axis'
        joint_state = JointState()




        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['swivel', 'tilt', 'periscope']
                joint_state.position = [swivel, tilt, height]

                # update transform
                # (moving in a circle with radius=2)
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = cos(angle)*2
                odom_trans.transform.translation.y = sin(angle)*2
                odom_trans.transform.translation.z = 0.7
                odom_trans.transform.rotation = \
                    euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)

                # Create new robot state
                tilt += tinc
                if tilt < -0.5 or tilt > 0.0:
                    tinc *= -1
                height += hinc
                if height > 0.2 or height < 0.0:
                    hinc *= -1
                swivel += degree
                angle += degree/4

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass



class Minimal_State_Publisher(Node):
    """
    This is a step by step recreation of state publisher to figure out what each part does..

    TIP: If you can't find the documentation for something with "py" at the end of the module/library's name, replace
    "py" with "cpp", and you will likely find the C++ documentation for how it works! 

    rclpy's documentation is weak, but rclcpp's is fuller!
    """
    def __init__(self):

        #Allows nodes to publish an subscribe?(TEST)
        #https://docs.ros2.org/alpha8/rclcpp_cpp_client_library_overview.html
        rclpy.init()

        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        #self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))


        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state
        tilt = 0.
        tinc = degree
        swivel = 0.
        angle = 0.
        height = 0.
        hinc = 0.005

        # Initialize nodes

        #the /odom node uses this class because ???
        odom_trans = TransformStamped()

        #nodes are split into header and child frames? the header is the name of the node it self, while the child is the name of nodes that are attached to it?
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        #joint_state = JointState()

        #rclpy is something that runs in the background? so checking if its ok is so if ros shuts it down this keeps running?
        while rclpy.ok():
            rclpy.spin_once(self)
            now = self.get_clock().now()

            #print("current time is now %s" % now)

            odom_trans.header.stamp = now.to_msg()
            #self.broadcaster.sendTransform(odom_trans)
            odom_trans.transform.translation.x = cos(angle)*2
            odom_trans.transform.translation.y = sin(angle)*2
            odom_trans.transform.translation.z = 0.7
            odom_trans.transform.rotation = \
            euler_to_quaternion(0, 0, angle + pi/2) # roll,pitch,yaw

            # send the joint state and transform
            #self.joint_pub.publish(joint_state)
            self.broadcaster.sendTransform(odom_trans)

            # Create new robot state
            tilt += tinc
            if tilt < -0.5 or tilt > 0.0:
                tinc *= -1
            height += hinc
            if height > 0.2 or height < 0.0:
                hinc *= -1
            swivel += degree
            angle += degree/4

            # This will adjust as needed per iteration
            loop_rate.sleep()


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def publish_model():
    """
    collects all of the stls that the model uses, creates a topic, populates it with relevant information, then publishes it. to ros2.
    """
    

    pass

def main():
    package_name = "model_pkg"
    print("the current file being executed is %s" % os.path.dirname(os.path.realpath(__file__)))
    #print("the current share directory for this package is: %s" % ament_index_python.get_package_share_directory(package_name) )
    
    node = Minimal_State_Publisher()
    #print(os.path.join('share/%s/launch') % "model_pkg", glob.glob('launch/*.py'))
    #print(os.path.join('share', package_name, 'launch'),
    #    glob.glob('launch/*.py'))
    #print('Hi from model_pkg.')

if __name__ == '__main__':
    main()
