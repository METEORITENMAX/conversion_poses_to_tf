from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    launch_path_to_tf_converter_node = Node(
        package='conversion_poses_to_tf',
        executable='path_to_tf_converter_node',
        name='path_to_tf_converter_node',
        output='screen',
        # parameters=[{
        #     'your_param_name': 'your_param_value'
        # }]
        remappings=[
            ('/path_topic', '/astar_planner_node/out/localPath'),
        ]
    )

    return LaunchDescription([
        launch_path_to_tf_converter_node
    ])