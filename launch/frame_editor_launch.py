import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, FindPackage  # Corrected import

def generate_launch_description():
    # Declare arguments for frame editor and RViz configurations
    frame_editor_config = LaunchConfiguration('frame_editor_config',
            default=PathJoinSubstitution([
            FindPackageShare('frame_editor'),
            'etc',
            'frames.yaml'
        ])
    )
    # frame_editor_config = LaunchConfiguration('frame_editor_config', default=FindPackageShare('frame_editor').find('etc/frames.yaml'))
    # print(frame_editor_config)

    
    # rviz_config = LaunchConfiguration('rviz_config', default=FindPackageShare('frame_editor').find('etc/frame_editor.rviz'))

    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument('frame_editor_config', default_value=frame_editor_config, description='Path to frames.yaml'),
        # DeclareLaunchArgument('rviz_config', default_value=rviz_config, description='Path to rviz configuration file'),

        # Launch the frame editor node
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='frame_editor_py',
            output='screen',
            arguments=[
                '--standalone', 'frame_editor',  # This will be passed as an argument to the rqt_gui executable
                '--args', '--load', frame_editor_config,  # Use LaunchConfiguration for dynamic argument substitution
                '--rate', '200'
            ]
        ),

        # Launch RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=[]#'-d', rviz_config]  # Use LaunchConfiguration for dynamic argument substitution
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

# import launch
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         # Declare arguments if needed
#         DeclareLaunchArgument('frame_editor_config', default_value='$(find frame_editor)/etc/frames.yaml', description='Path to frames.yaml'),
#         DeclareLaunchArgument('rviz_config', default_value='$(find frame_editor)/etc/frame_editor.rviz', description='Path to rviz configuration file'),

#         # Launch the frame editor node
#         Node(
#             package='rqt_gui',
#             executable='rqt_gui',
#             name='frame_editor_py',
#             output='screen',
#             arguments=[
#                 '--standalone', 'frame_editor',
#                 '--args', '--load', "$(arg frame_editor_config)",
#                 '--rate', '200'
#             ]
#         ),

#         # Launch rviz node
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz',
#             output='screen',
#             arguments=['-d', "$(arg rviz_config)"]
#         ),
        
#         Node(
#             package='tf2_ros',
#             executable='static_transform_publisher',
#             name='static_map_to_world',
#             output='screen',
#             arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']  # Translation (x, y, z) and rotation (roll, pitch, yaw)
#         )
#     ])
