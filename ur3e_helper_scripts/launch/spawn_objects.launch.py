import os
import time
import xacro
import random
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # get directory information
    package = "ur3e_helper_scripts"
    package_path = get_package_share_directory(package)
    
    # load spawn object description file
    xacro_dir_path = os.path.join(package_path, "xacro")
    xacro_file_path = os.path.join(xacro_dir_path, "grasp_object.xacro")

    # set grasp object properties [colors must be exactly as defined in the object_props.xacro file]
    colors = ["grey", "darkgrey", "white", "flatblack", "black", "red", "redbright", "green", "blue",
              "skyblue", "yellow", "zincyellow", "darkyellow", "purple", "turquoise", "orange", "indigo"]
    random.shuffle(colors)
    grasp_objects = [{"size": (+0.0650, +0.0250, +0.0750), "xyz": (+5.7000, -3.4500, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)},
                     {"size": (+0.0600, +0.0250, +0.0750), "xyz": (+5.7000, -3.5625, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)},
                     {"size": (+0.0250, +0.0250, +0.0750), "xyz": (+5.7000, -3.6550, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)},
                     {"size": (+0.0200, +0.0250, +0.0750), "xyz": (+5.7000, -3.7275, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)},
                     {"size": (+0.0700, +0.0250, +0.0750), "xyz": (+5.6250, -3.4500, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)},
                     {"size": (+0.0550, +0.0250, +0.0750), "xyz": (+5.6250, -3.5625, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)},
                     {"size": (+0.0300, +0.0250, +0.0750), "xyz": (+5.6250, -3.6550, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)},
                     {"size": (+0.0150, +0.0250, +0.0750), "xyz": (+5.6250, -3.7275, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)},
                     {"size": (+0.0750, +0.0250, +0.0750), "xyz": (+5.5500, -3.4500, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)},
                     {"size": (+0.0500, +0.0250, +0.0750), "xyz": (+5.5500, -3.5625, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)},
                     {"size": (+0.0350, +0.0250, +0.0750), "xyz": (+5.5500, -3.6550, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)},
                     {"size": (+0.0100, +0.0250, +0.0750), "xyz": (+5.5500, -3.7275, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)},
                     {"size": (+0.0800, +0.0250, +0.0750), "xyz": (+5.4750, -3.4500, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)},
                     {"size": (+0.0450, +0.0250, +0.0750), "xyz": (+5.4750, -3.5625, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)},
                     {"size": (+0.0400, +0.0250, +0.0750), "xyz": (+5.4750, -3.6550, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)},
                     {"size": (+0.0050, +0.0250, +0.0750), "xyz": (+5.4750, -3.7275, +0.9550), "rpy": (+0.0000, +0.0000, +1.5708)}]
    
    # delete temporary xml files
    for temp_file in os.listdir(xacro_dir_path):
        if (temp_file.startswith("temp_") and temp_file.endswith(".xml")):
            temp_file_path = os.path.join(xacro_dir_path, temp_file)
            os.remove(temp_file_path)
        else:
            pass
    
    # spawn grasp objects
    entity_id = int(str(time.time())[0:-8])
    spawn_nodes = []
    for n in range(len(grasp_objects)):
        size = grasp_objects[n]["size"]
        xyz = grasp_objects[n]["xyz"]
        rpy = grasp_objects[n]["rpy"]
        xacro_args = {"mass": "0.050", "xsize": str(size[0]), "ysize": str(size[1]), "zsize": str(size[2]), "color": colors[n]}
        processed_xacro_data = xacro.process_file(input_file_name=xacro_file_path, mappings=xacro_args)
        xacro_xml_data = processed_xacro_data.toxml()
        temp_xml_path = os.path.join(xacro_dir_path, "temp_" + str(entity_id) + ".xml")
        with open(temp_xml_path, "w") as temp_xml:
            temp_xml.write(xacro_xml_data)
        entity_id += 1
        spawn_nodes.append(
            Node(package='gazebo_ros', 
                executable='spawn_entity.py', 
                name='grasp_object_spawn_entity_' + str(entity_id),
                output='screen', 
                emulate_tty=True, 
                arguments=['-entity', "grasp_object_" + str(entity_id),
                            '-x', str(xyz[0]), '-y', str(xyz[1]), '-z', str(xyz[2]),
                            '-R', str(rpy[0]), '-P', str(rpy[1]), '-Y', str(rpy[2]),
                            '-file', temp_xml_path],
            )
        )

    return LaunchDescription(spawn_nodes)

# End of Code
