from launch import LaunchDescription
from launch_ros.actions import Node
import ament_index_python

package_name = 'model_pkg'
share_directory = ament_index_python.packages.get_package_share_directory(package_name)


def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='model_pkg',
            executable='model',
            output='screen',
            parameters=[],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguements=['-d', share_directory + '/rviz/rviz_config_test.rviz'],
        ),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            output='screen',
            parameters=[],
        ),
    ])
