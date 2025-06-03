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
    moveit_config = MoveItConfigsBuilder("name", package_name="real_moveit_config").to_moveit_configs()

    # declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                                 description='False: Use Real Robot Clock & True: Use Simulation Clock')
    declare_use_cube = DeclareLaunchArgument(name='use_cube', default_value='False',
                                             description='True: Use Cube as Grasp Object')
    declare_use_block = DeclareLaunchArgument(name='use_block', default_value='True',
                                              description='True: Use Block as Grasp Object')
    declare_lying = DeclareLaunchArgument(name='lying', default_value='False',
                                          description='True: Indicates that Object is Lying / Fallen')
    declare_standing = DeclareLaunchArgument(name='standing', default_value='True',
                                             description='True: Indicates that Object is Standing / Upright')
    # block side description: longest side is height, wide side is length, small side is breadth
    # since target pose is standing, the block can not be gripped by height, so either length or breadth
    # example block dimension: 0.04 x 0.02 x 0.08 => Length x Breadth x Height
    declare_slim_grip = DeclareLaunchArgument(name='slim_grip', default_value='True',
                                              description='True: Grip the Block by the Slim Side')
    declare_wide_grip = DeclareLaunchArgument(name='wide_grip', default_value='False',
                                              description='True: Grip the Block by the Wide Side')

    # use launch configuration in nodes
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_cube = LaunchConfiguration('use_cube')
    use_block = LaunchConfiguration('use_block')
    lying = LaunchConfiguration('lying')
    standing = LaunchConfiguration('standing')
    slim_grip = LaunchConfiguration('slim_grip')
    wide_grip = LaunchConfiguration('wide_grip')

    # moveit cpp executable
    moveit_cpp_node = Node(
        name="setup_grasp_object",
        package="ur3e_helper_scripts",
        executable="setup_grasp_object",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time},
            {'use_cube': use_cube}, {'use_block': use_block},
            {'lying': lying}, {'standing': standing},
            {'slim_grip': slim_grip}, {'wide_grip': wide_grip},
        ],
    )

    return LaunchDescription(
        [declare_use_sim_time, 
         declare_use_cube, 
         declare_use_block, 
         declare_standing, 
         declare_lying, 
         declare_wide_grip, 
         declare_slim_grip, 
         moveit_cpp_node]
    )

# End of Code