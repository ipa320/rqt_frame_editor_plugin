import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, FindPackage 

def generate_launch_description():
    frame_editor_config = LaunchConfiguration('frame_editor_config',
            default=PathJoinSubstitution([
            FindPackageShare('frame_editor'),
            'etc',
            'frames.yaml'
        ])
    )
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('frame_editor'),
        'etc',
        'frame_editor.rviz'
    ])
        
    return LaunchDescription([
        DeclareLaunchArgument('frame_editor_config', default_value=frame_editor_config, description='Path to frames.yaml'),
        DeclareLaunchArgument('rate', default_value='200', description='Rate of processing'),
        Node(
            package='frame_editor',
            executable='editor.py',
            name='frame_editor_py',
            output='screen',
            arguments=[
                '--load', frame_editor_config,
            ]
        ),

        # Launch RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),

        # Launch static transform publisher node
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_world',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']  # Translation (x, y, z) and rotation (roll, pitch, yaw)
        )
    ])
