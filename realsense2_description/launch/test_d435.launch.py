"""
Launch file to test generation of a Intel Realsense RGBD camera in Gazebo Harmonic.
The world contains three generic objects with an Intel realsense camera observing them

Author: Azmyin Md. Kamal
Date: 01/11/2025 

Based on: https://github.com/Mechazo11/rbkairos_description_ros2/blob/main/launch/view_robot.launch.py

AI Tool: Claude Sonnet 3.5
"""

# imports
# ROS 2
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,FindExecutable
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.actions import LogInfo


# Declare arguments
ARGUMENTS = [
    DeclareLaunchArgument('namespace',default_value='',
        description='Robot namespace'
    ),
    DeclareLaunchArgument('camera_name',default_value='realsense_d435_',
        description='Unique name for this realsense camera'
    ),
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument(name='config', default_value='view_robot.rviz',
        description='Rviz configuration'
    ),
]

def launch_setup(context, *args, **kwargs):
    """Initialize and configure the launch setup for a ROS 2 application."""
    # Parse runtime arguments
    camera_name = LaunchConfiguration('camera_name')
    namepspace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Directories
    pkg_realsense2_description = FindPackageShare('realsense2_description')
    config_rviz = PathJoinSubstitution(
        [pkg_realsense2_description, 'rviz', LaunchConfiguration('config')]
    )
    
    # Careful with the package, directory and urdf file names
    camera_description = ParameterValue(
    Command([
        'xacro ',
        PathJoinSubstitution([
            pkg_realsense2_description,
            'urdf',
            'visualize_d435.urdf.xacro'
        ])
    ]),
    value_type=str
    )

    log_urdf_path = LogInfo(msg=["Resolved robot URDF path: ", camera_description])

    group_action_view_model_in_rviz = GroupAction([
        # log_urdf_path, # Debug
        PushRosNamespace(namepspace),
        
        # Spin up RViz
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', config_rviz],
             parameters=[{'use_sim_time': use_sim_time}],
             remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')
             ],
             output='screen'),
        # Spin up Gazebo
    ])


    return [
        group_action_view_model_in_rviz
    ]

def generate_launch_description():
    """Generate launch description."""
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld