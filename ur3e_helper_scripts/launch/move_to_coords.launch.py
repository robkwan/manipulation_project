import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # load moveit configs for the robot model
    # moveit_config = MoveItConfigsBuilder("name", package_name="sim_ur3e_config").to_moveit_configs()
    moveit_config = MoveItConfigsBuilder("name", package_name="real_ur3e_config").to_moveit_configs()

    # declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                                 description='False: Use Real Robot Clock & True: Use Simulation Clock')
    declare_x = DeclareLaunchArgument(name='x', default_value='0.250',
                                      description='Initial Position of Grasp Object - X Axis')
    declare_y = DeclareLaunchArgument(name='y', default_value='0.250',
                                      description='Initial Position of Grasp Object - Y Axis')
    declare_z = DeclareLaunchArgument(name='z', default_value='0.250',
                                      description='Initial Position of Grasp Object - Z Axis')
    declare_steps = DeclareLaunchArgument(name='steps', default_value='once',
                                      description='One Step Movement or Three Step Movement')

    # use launch configuration in nodes
    use_sim_time = LaunchConfiguration('use_sim_time')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    steps = LaunchConfiguration('steps')

    # moveit cpp executable
    moveit_cpp_node = Node(
        name="move_to_coords",
        package="ur3e_helper_scripts",
        executable="move_to_coords",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time},
            {'x': x}, {'y': y}, {'z': z},
            {'steps': steps},
        ],
    )

    return LaunchDescription(
        [declare_use_sim_time, 
         declare_x,
         declare_y,
         declare_z,
         declare_steps,
         moveit_cpp_node]
    )

# End of Code