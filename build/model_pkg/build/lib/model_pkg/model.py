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

# numpy.linalg.eigvals
"""
# Helpful resources:
# https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html

SDF FORMAT(FOR GAZEBO):
---
    http://sdformat.org/spec
"""


class Robot(Node):
    """
    class for everything related to a robot
    """

    def __init__(self):

        # this needs to be initialized first because ???
        rclpy.init()

        # robot uses information from state_publisher node?
        super().__init__('state_publisher')
        self.qos_profile = QoSProfile(depth=10)
        """
        something something ros1 to ros2 bridge. I have no clue what this does but code breaks when I remove it.
        
        look here for documentation:
        ``https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html``
        """

        self.odom_trans = TransformStamped()
        """
        /odom or odom node.

        This node is used by /robot_state_publisher

        odometry information like 2d lidar like for /scan /econoder_(l)(r) should be sent to this node during real time.
        otherwise, it should be generated via sim. 
        """
        self.joint_state = JointState()
        """
        /<insert joint> node's parent object.
        this encompases all joint nodes /left_wheel_joint, /right_wheel_joint. etc... If it rotates, swivels, etc.. its info is here.
        """

        # robot state variables(current bot rotation, current angle of wheels, etc...)
        #tilt = 0.
        #tinc = degree
        #swivel = 0.
        self.angle = 0.
        #height = 0.
        #hinc = 0.005
        self.left_wheel_radians = 0.0
        self.right_wheel_radians = 0.0

        # set default info for transforms
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'base_link'

        # initalize node publishers for robot

        # it would be nice is odom_pub was phonetically the same as joint_pub, and not a completly different publishing mechanism...
        self.odom_pub = TransformBroadcaster(self, qos=self.qos_profile)
        self.joint_pub = self.create_publisher(
            JointState, 'joint_states', self.qos_profile)

    def simulate_self(self):
        """
        simulate info about this robot for Gazebo/RVIZ2, other ros2 node readers to pickup

        this assumes the other robot being simulated is this one.
        """
        loop_rate = self.create_rate(30)
        """
        rate at which the simulation runs in Hz
        """
        
        #rclpy.init()
        while rclpy.ok():
            rclpy.spin_once(self)

            now = self.get_clock().now()

            self.joint_state.header.stamp = now.to_msg()
            self.joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
            self.joint_state.position = [self.left_wheel_radians, self.right_wheel_radians]

            self.odom_trans.header.stamp = now.to_msg()
            self.odom_trans.transform.translation.x = self.odom_trans.transform.translation.x + \
                0.01  # cos(angle)*2
            self.odom_trans.transform.translation.y = 0.0  # sin(angle)*2
            self.odom_trans.transform.translation.z = 0.0
            self.odom_trans.transform.rotation = \
                self.euler_to_quaternion(0, 0, self.angle + pi/2)  # roll,pitch,yaw

            # send the joint state and transform
            self.joint_pub.publish(self.joint_state)
            self.odom_pub.sendTransform(self.odom_trans)

            # set turn for left and right wheels based on <INSERT SOMETHING THAT FETCHES THAT DATA HERE>
            self.left_wheel_radians = self.left_wheel_radians + 0.1
            self.right_wheel_radians = self.right_wheel_radians + 0.1

            loop_rate.sleep()

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - \
            cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + \
            sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - \
            sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + \
            sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def orient_to(self, cords=[0.0, 0.0, 0.0]):
        pass

    def move_to(self, cords=[0.0, 0.0, 0.0]):
        """
        orient the robot in the direction to move to, 
        and drive to that location
        """


class Minimal_State_Publisher(Node):
    """
    This is a step by step recreation of state publisher to figure out what each part does..

    TIP: If you can't find the documentation for something with "py" at the end of the module/library's name, replace
    "py" with "cpp", and you will likely find the C++ documentation for how it works! 

    rclpy's documentation is weak, but rclcpp's is fuller!
    """

    def __init__(self):

        # Allows nodes to publish an subscribe?(TEST)
        # https://docs.ros2.org/alpha8/rclcpp_cpp_client_library_overview.html

        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(
            JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state variables(current bot rotation, current angle of wheels, etc...)
        #tilt = 0.
        #tinc = degree
        #swivel = 0.
        angle = 0.
        #height = 0.
        #hinc = 0.005
        left_wheel_radians = 0.0
        right_wheel_radians = 0.0

        # Initialize nodes

        # the /odom node uses this class because ???
        odom_trans = TransformStamped()

        # nodes are split into header and child frames? the header is the name of the node it self, while the child is the name of nodes that are attached to it?
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'

        odom_trans.transform.translation.x = 0.0  # cos(angle)*2
        odom_trans.transform.translation.y = 0.0  # sin(angle)*2
        odom_trans.transform.translation.z = 0.0
        joint_state = JointState()

        # rclpy is something that runs in the background? so checking if its ok is so if ros shuts it down this keeps running?
        while rclpy.ok():
            rclpy.spin_once(self)
            now = self.get_clock().now()

            #print("current time is now %s" % now)

            joint_state.header.stamp = now.to_msg()
            joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
            joint_state.position = [left_wheel_radians, right_wheel_radians]

            odom_trans.header.stamp = now.to_msg()
            odom_trans.transform.translation.x = odom_trans.transform.translation.x + \
                0.01  # cos(angle)*2
            odom_trans.transform.translation.y = 0.0  # sin(angle)*2
            odom_trans.transform.translation.z = 0.0
            odom_trans.transform.rotation = \
                euler_to_quaternion(0, 0, angle + pi/2)  # roll,pitch,yaw

            # send the joint state and transform
            self.joint_pub.publish(joint_state)
            self.broadcaster.sendTransform(odom_trans)

            # set turn for left and right wheels based on <INSERT SOMETHING THAT FETCHES THAT DATA HERE>
            left_wheel_radians = left_wheel_radians + 0.1
            right_wheel_radians = right_wheel_radians + 0.1
            # Create new robot state
            #tilt += tinc
            # if tilt < -0.5 or tilt > 0.0:
            #    tinc *= -1
            #height += hinc
            # if height > 0.2 or height < 0.0:
            #    hinc *= -1
            #swivel += degree
            #angle += degree/4

            # This will adjust as needed per iteration
            loop_rate.sleep()


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - \
        cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + \
        sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - \
        sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + \
        sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    package_name = "model_pkg"
    print("the current file being executed is %s" %
          os.path.dirname(os.path.realpath(__file__)))

    #node = Minimal_State_Publisher()
    diff_bot = Robot()
    diff_bot.simulate_self()

    #print("the current share directory for this package is: %s" % ament_index_python.get_package_share_directory(package_name) )

    #node = Minimal_State_Publisher()
    #print(os.path.join('share/%s/launch') % "model_pkg", glob.glob('launch/*.py'))
    # print(os.path.join('share', package_name, 'launch'),
    #    glob.glob('launch/*.py'))
    #print('Hi from model_pkg.')


if __name__ == '__main__':
    main()
