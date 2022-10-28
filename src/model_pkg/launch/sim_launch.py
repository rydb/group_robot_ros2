from launch import LaunchDescription
from launch_ros.actions import Node

#this launch gathers all of the packages and their nodes required to launch a simulated version of the robot in rviz

#output means output goes to terminal that executed this launch file
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='model_pkg',
            executable='model',
            output="screen"
        ),
        Node(
            package="rviz2",
            executable='rviz2',
            output="screen"
        )
    ])