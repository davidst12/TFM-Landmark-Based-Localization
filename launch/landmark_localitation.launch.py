from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
   return LaunchDescription([
        Node(
            package='tfm_landmark_based_localization_package',
            executable='fake_pole_detection_node',
            name='fake_pole_detection_node',
            parameters=[os.path.join(
                get_package_prefix('tfm_landmark_based_localization_package'), '../../src/tfm_landmark_based_localization_package',
                'config', 'global_parameters.yaml'
            )],
            output='screen'
),
        Node(
            package='tfm_landmark_based_localization_package',
            executable='landmarks_graph_location_node',
            name='landmarks_graph_location_node',
            parameters=[os.path.join(
                get_package_prefix('tfm_landmark_based_localization_package'), '../../src/tfm_landmark_based_localization_package',
                'config', 'global_parameters.yaml'
            )],
            output='screen',
            prefix=["gnome-terminal --title='Landmarks Graph Location Node' -- "]
        ),
        Node(
            package='tfm_landmark_based_localization_package',
            executable='location_statistics_node',
            name='location_statistics_node',
            output='screen',
            prefix=["gnome-terminal --title='Location Statistics Node' -- "]
        ),
   ])