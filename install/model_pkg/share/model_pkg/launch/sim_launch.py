from launch import LaunchDescription
from launch_ros.actions import Node
import ament_index_python
#this launch gathers all of the packages and their nodes required to launch a simulated version of the robot in rviz

#output means output goes to terminal that executed this launch file

package_name = "model_pkg"
def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package=package_name,
            executable='model',
            output="screen",
            parameters=[],
        ),
        Node(
            package="rviz2",
            executable='rviz2',
            output="screen",
            arguments=["-d", "%s/rviz/rviz_config_test.rviz" % ament_index_python.packages.get_package_share_directory(package_name)],
        ),
        Node(
            package="rqt_gui",
            executable="rqt_gui",
            output="screen",
            parameters=[]
        )
    ])