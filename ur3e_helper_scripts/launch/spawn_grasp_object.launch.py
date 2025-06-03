import os
import time
import xacro
import random
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    # get directory information
    package = "ur3e_helper_scripts"
    package_path = get_package_share_directory(package)

    # load spawn object description file
    xacro_dir_path = os.path.join(package_path, "xacro")
    xacro_file_path = os.path.join(xacro_dir_path, "grasp_object.xacro")

    # use launch configuration in nodes
    mass = LaunchConfiguration('mass').perform(context)
    xsize = LaunchConfiguration('xsize').perform(context)
    ysize = LaunchConfiguration('ysize').perform(context)
    zsize = LaunchConfiguration('zsize').perform(context)
    color = LaunchConfiguration('color').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    R = LaunchConfiguration('R').perform(context)
    P = LaunchConfiguration('P').perform(context)
    Y = LaunchConfiguration('Y').perform(context)

    # delete temporary xml files
    for temp_file in os.listdir(xacro_dir_path):
        if (temp_file.startswith("temp_") and temp_file.endswith(".xml")):
            temp_file_path = os.path.join(xacro_dir_path, temp_file)
            os.remove(temp_file_path)
        else:
            pass

    # spawn grasp object
    entity_id = int(str(time.time())[0:-8])
    colors = ["grey", "darkgrey", "white", "flatblack", "black", "red", "redbright", "green", "blue",
              "skyblue", "yellow", "zincyellow", "darkyellow", "purple", "turquoise", "orange", "indigo"]
    if (color == "random"):
        color = random.choice(colors)
    else:
        pass
    xacro_args = {"mass": mass, "xsize": xsize, "ysize": ysize, "zsize": zsize, "color": color}
    processed_xacro_data = xacro.process_file(input_file_name=xacro_file_path, mappings=xacro_args)
    xacro_xml_data = processed_xacro_data.toxml()
    temp_xml_path = os.path.join(xacro_dir_path, "temp_" + str(entity_id) + ".xml")
    with open(temp_xml_path, "w") as temp_xml:
        temp_xml.write(xacro_xml_data)
    spawn_node = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        name='grasp_object_spawn_entity_' + str(entity_id),
        output='screen', emulate_tty=True, 
        arguments=[
            '-entity', "grasp_object_" + str(entity_id),
            '-x', x, '-y', y, '-z', z, '-R', R, '-P', P, '-Y', Y,
            '-file', temp_xml_path,
        ],
    )

    return [spawn_node, ]

def generate_launch_description():

    # declare launch arguments
    declare_mass = DeclareLaunchArgument(name='mass', default_value='0.0500', description='Grasp Object Mass in Kilograms')
    declare_xsize = DeclareLaunchArgument(name='xsize', default_value='0.0150', description='Grasp Object X Size in Meters')
    declare_ysize = DeclareLaunchArgument(name='ysize', default_value='0.0250', description='Grasp Object Y Size in Meters')
    declare_zsize = DeclareLaunchArgument(name='zsize', default_value='0.0750', description='Grasp Object Z Size in Meters')
    declare_color = DeclareLaunchArgument(name='color', default_value='random', description='Grasp Object Color as in Xacro File')
    declare_x = DeclareLaunchArgument(name='x', default_value='+5.2800', description='Grasp Object Position in X Axis in Meters')
    declare_y = DeclareLaunchArgument(name='y', default_value='-3.8400', description='Grasp Object Position in Y Axis in Meters')
    declare_z = DeclareLaunchArgument(name='z', default_value='+0.9550', description='Grasp Object Position in Z Axis in Meters')
    declare_R = DeclareLaunchArgument(name='R', default_value='+0.0000', description='Grasp Object Orientation in X Axis in Radians')
    declare_P = DeclareLaunchArgument(name='P', default_value='+0.0000', description='Grasp Object Orientation in Y Axis in Radians')
    declare_Y = DeclareLaunchArgument(name='Y', default_value='+0.0000', description='Grasp Object Orientation in Z Axis in Radians')

    return LaunchDescription(
        [declare_mass, declare_xsize, declare_ysize, declare_zsize, declare_color,
         declare_x, declare_y, declare_z, declare_R, declare_P, declare_Y,
         OpaqueFunction(function = launch_setup)]
    )

# End of Code
