from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('control_systems')
    urdf = os.path.join(pkg_path,'urdf','diff_drive_vel.urdf')

#    # Launch Gazebo
#     gazebo = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([
#             os.path.join(
#                 FindPackageShare('gazebo_ros').find('gazebo_ros'),
#                 'launch',
#                 'gazebo.launch.py'
#             )
#         ]),
#     )
 
                        


    return LaunchDescription([
        ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]
        ),
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        #     arguments=[urdf]
        # ),
       
         # Spawn the robot entity in Gazebo simulation from the URDF topic
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='robot_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "diff_drive_bot"]),

        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=["velocity_controller"]),
        
        Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=["joint_state_broadcaster"]),


    ])
