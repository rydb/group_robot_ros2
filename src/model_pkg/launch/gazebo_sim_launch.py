from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import ament_index_python

package_name = 'model_pkg'
share_directory = ament_index_python.packages.get_package_share_directory(package_name)

urdf = share_directory + '/urdf/diff_bot.urdf.xml'
def generate_launch_description():
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    return LaunchDescription([
        ExecuteProcess(
            command=['gazebo', '--verbose', '-s', ],
            output='screen',
        ),
        Node(
            package='model_pkg',
            executable='model',
            output='screen',
            parameters=[],
        ),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            output='screen',
            parameters=[],
        ),
        Node(
            package= 'robot_state_publisher' ,
            executable= 'robot_state_publisher' ,
            name= 'robot_state_publisher' ,
            output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        ),
    ])