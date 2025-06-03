import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # load moveit configs for the robot model
    # moveit_config = MoveItConfigsBuilder("name", package_name="sim_ur3e_config").to_moveit_configs()
    moveit_config = MoveItConfigsBuilder("name", package_name="real_ur3e_config").to_moveit_configs()

    # declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                                 description='False: Use Real Robot Clock & True: Use Simulation Clock')
    
    # use launch configuration in nodes
    use_sim_time = LaunchConfiguration('use_sim_time')

    # moveit cpp executable
    moveit_cpp_node = Node(
        name="display_current_pose",
        package="ur3e_helper_scripts",
        executable="display_current_pose",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription(
        [declare_use_sim_time, 
         moveit_cpp_node]
    )

# End of Code