import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare use_sim_time launch arg
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    #spots_config = os.path.join(get_package_share_directory('nav2_apps'),'config','spot-list.yaml')

    static_tf_pub_node = Node(
        package='object_detection',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        parameters = [{'use_sim_time':use_sim_time}]
    )

    obj_det_node = Node(
        package = 'object_detection',
        name = 'object_detection',
        executable = 'object_detection',
        output = 'screen',
        parameters = [{'use_sim_time':use_sim_time}]
    )
   
    return LaunchDescription([ 
    
        declare_use_sim_time,

        obj_det_node,

    ])