README
------


UR3e Helper Scripts Package
---------------------------

This is a ROS2 and MoveIt2 based package, created to assist MasterClass students of The Construct.

This package contains programs and scripts to assist with Checkpoints that use UR3e Real Robot.
These helper scripts aims to reduce block setting up time during real-robot sessions.

This package was created with the notion to assist students to setup a block at the initial block "pick-up" spot for pick-and-place tasks using UR3e Robot itself.
This eliminates the need for some person in the lab to physically set the block at the initial "pick" position every time.

The programs were made in a short time for quick use. Improvements will be released in the future versions.


Contents of Package:
--------------------

1. Source Files:
    1. display_current_pose.cpp - This program displays the current pose (position & orientation) of the UR3 Arm and Gripper only (and does nothing else).
	2. move_to_blocks.cpp - This program positions the UR3e Arm close towards the area of blocks.
    3. move_to_coords.cpp - This program positions the UR3e Arm at the target pose coordinates.
    4. reset_grasp_object.cpp - This program takes the object from the pick pose and places back near the group of blocks.
	5. setup_grasp_object.cpp - This program picks up the block and brings it to the actual "pick" coordinates and places it at the spot with proper orientation.

2. Launch Files:
	1. display_current_pose.launch.py - This launch file launches the display_current_pose.cpp program executable. Requires no launch arguments.
    2. move_to_blocks.launch.py - This launch file launches the move_to_blocks.cpp program executable. Requires no launch arguments.
    3. move_to_coords.launch.py - This launch file launches the move_to_coords.cpp program executable. Requires launch arguments.
	4. reset_grasp_object.launch.py - This launch file launches the reset_grasp_object.cpp program executable. Requires launch arguments.
    5. setup_grasp_object.launch.py - This launch file launches the setup_grasp_object.cpp program executable. Requires launch arguments.
    6. spawn_grasp_object.launch.py - This launch file spawns single grasp object into the simulation. Requires launch arguments.
    7. spawn_objects.launch.py - This launch file spawns multiple grasp objects into the simulation. Requires no launch arguments.

3. Xacro Files:
    1. grasp_object.xacro - This xacro file contains the Description of the Grasp Object.
    2. object_props.xacro - This xacro file contains the Properties of the Grasp Object - Inertial, Collision, Visual and Materials.

4. Commands.txt - Contains commands ready for copy-pasting in the terminal.

5. Notes.txt - Contains information about the simulation and real robot setup.

6. Media Files - Images and Videos
    - Demonstrates each process to give you an idea of how the programs work.
    - Videos [mp4] are provided due to size limitations of Animations [gif].
    - Videos CANNOT be viewed on the online IDE, please download them to watch.


How to Use:
-----------

Usually, you will find a block positioned at the initial "pick-up" spot, where it is marked with a X-in-a-box symbol.
But in some cases, you might not find a block positioned at that place. The helper programs in this package will help you setup a block.
But beware, this is not a magical program that will automatically set a block for you from anywhere! It also requires some input from you.

You will need at least 4 terminals to run this program:
1. Use the first terminal, to launch the "move_group" [MoveIt framework] program for the real robot.
2. Use the second terminal, to launch the "moveit_rviz" [MoveIt RViz2 GUI] program for the real robot.
3. Use the third terminal to launch the helper scripts - "move_to_blocks" and "setup_grasp_object".
4. Use the fourth terminal to launch your pick-and-place program that you have designed.

Usage Steps:
------------
1. Launch "move_group" node for Real Robot
2. Launch "moveit_rviz" node for Real Robot
3. Launch "move_to_blocks" program for Real Robot
4. Use the RViz MoveIt GUI to position the gripper above the object to grasp. DO NOT close the gripper manually!
5. Launch "setup_grasp_object" program with relevant launch arguments.

IMPORTANT NOTE:
---------------
You need to change the package name in the launch files to the name of the MoveIt2 Configuration Package that you have created for the Simulation and the Real Robot(s).
You also need to change the PLANNING_GROUP names from "ur3_arm" and "gripper" to the values that you have defined in your MoveIt2 Configuration Package.
You must change the names in the following two lines in all the C++ program codes present in the "ur3e_helper_scripts/src" directory.
1. static const std::string PLANNING_GROUP_UR3_ARM = "ur3_arm"; // replace "ur3_arm" as required
2. static const std::string PLANNING_GROUP_GRIPPER = "gripper"; // replace "gripper" as required

Geometry Conventions:
---------------------

The block is always setup in a "standing" pose. Therefore, gripping the block by the longest side of the block is not viable for block setup.

Block dimension convention: Given the fact that the block will NEVER be gripped by the longest side, we will use the other two sides for gripping.
The longest side is the Height. The smallest side is the Breadth. The wide side (third side) is the Length.
Example: 0.04 x 0.02 x 0.08 : Here, 0.08 is Height [not Length], 0.04 is Length and 0.02 is Breadth.

You will find two types of grasping / gripping in the program - "wide_grip" and "slim_grip".
1. wide_grip - Sets the gripper to grasp the block by the Length side - Sets the gripper gap to block Length measure - Gripper gap = Block Length.
2. slim_grip - Sets the gripper to grasp the block by the Breadth Side - Sets the gripper gap to block Breadth measure - Gripper gap = Block Breadth.
You can choose the type of gripping before you start the "setup_grasp_object" program in the launch command line.
NOTE: "wide_grip" and "slim_grip" will be omitted when Cube is chosen as grasp object instead of Block.


Some Advice on Usage:
---------------------
PLEASE TRY EVERYTHING IN THE SIMULATION BEFORE USING IT ON THE REAL ROBOT TO ENSURE THERE ARE NO ACCIDENTS !!!
Please make sure to set the UR3e Robot ALWAYS in a "Home" position BEFORE you exit the Real Robot Lab session.
Leaving the robot arm pointing outside is not encouraged and it is not a good practice.


Program Launch Arguments:
-------------------------

display_current_pose:
	- use_sim_time: Defaults to False
		- True -> Use Simulation Clock.
		- False -> Use Real Robot Clock.

move_to_blocks:
	- use_sim_time: Defaults to False
		- True -> Use Simulation Clock.
		- False -> Use Real Robot Clock.

move_to_coords:
	- use_sim_time: Defaults to False
		- True -> Use Simulation Clock.
		- False -> Use Real Robot Clock.
    - x: Value for Target Pose X Coordinate -> Defaults to +0.250
    - y: Value for Target Pose Y Coordinate -> Defaults to +0.250
    - z: Value for Target Pose Z Coordinate -> Defaults to +0.250
    - steps: Defaults to once
        - once -> Uses one step move to reach target XYZ pose.
        - xyz or xzy or yxz or yzx or zxy or zyx -> Uses three steps move in the given order.

reset_grasp_object:
    - use_sim_time: Defaults to False
		- True -> Use Simulation Clock.
		- False -> Use Real Robot Clock.
    - x: Value for Target Pose X Coordinate -> Defaults to +0.150
    - y: Value for Target Pose Y Coordinate -> Defaults to +0.250
    - z: Value for Target Pose Z Coordinate -> Defaults to +0.200

setup_grasp_object:
	- use_sim_time: Defaults to False
		- True -> Use Simulation Clock.
		- False -> Use Real Robot Clock.
	- use_cube: Defaults to False
		- True: Uses Cube as Grasp Object. Sets use_block to False.
		- False: Uses Block as Grasp Object. Sets use_cube to True.
	- use_block: Defaults to True
		- True: Uses Block as Grasp Object. Sets use_cube to False.
		- False: Uses Cube as Grasp Object. Sets use_block to True.
	- lying: Defaults to False
		- True: Indicates that Grasp Object is Lying Down. Sets standing to False.
		- False: Indicates that Grasp Object is Standing Up. Sets standing to True.
	- standing: Defaults to True
		- True: Indicates that Grasp Object is Standing Up. Sets lying to False.
		- False: Indicates that Grasp Object is Lying Down. Sets lying to True.
	- slim_grip: Defaults to True
		- True: Sets Gripper Gap distance value to Block Breadth measure. Sets wide_grip to False.
		- False: Sets Gripper Gap distance value to Block Length measure. Sets wide_grip to True.
	- wide_grip: Defaults to False
		- True: Sets Gripper Gap distance value to Block Length measure. Sets slim_grip to False.
		- False: Sets Gripper Gap distance value to Block Breadth measure. Sets slim_grip to True.

spawn_grasp_object:
    - mass: Grasp Object Mass in Kilograms -> Defaults to 0.0500
    - xsize: Grasp Object X Size in Meters -> Defaults to 0.0150
    - ysize: Grasp Object Y Size in Meters -> Defaults to 0.0250
    - zsize: Grasp Object Z Size in Meters -> Defaults to 0.0750
    - color: Grasp Object Color as in Xacro File -> Defaults to random
    - x: Grasp Object Position in X Axis in Meters -> Defaults to +5.2800
    - y: Grasp Object Position in Y Axis in Meters -> Defaults to -3.8400
    - z: Grasp Object Position in Z Axis in Meters -> Defaults to +0.9550
    - R: Grasp Object Orientation in X Axis in Radians -> Defaults to +0.0000
    - P: Grasp Object Orientation in Y Axis in Radians -> Defaults to +0.0000
    - Y: Grasp Object Orientation in Z Axis in Radians -> Defaults to +0.0000
    - Works ONLY for Simulation.
    - Will NOT Work for Real Robot.

spawn_objects:
    - No Launch Arguments Required.
    - Works ONLY for Simulation.
    - Will NOT Work for Real Robot.


Launch Commands:
----------------

move_group node:
----------------
ros2 launch <real_moveit2_config> move_group.launch.py

moveit_rviz node:
-----------------
ros2 launch <real_moveit2_config> moveit_rviz.launch.py

Display Current Poses of UR3 Arm and Gripper [Real Robot]:
----------------------------------------------------------
ros2 launch ur3e_helper_scripts display_current_pose.launch.py

Position the UR3 Arm near the Blocks [Real Robot]:
---------------------------------------------------
ros2 launch ur3e_helper_scripts move_to_blocks.launch.py

Position the UR3 Arm at a given XYZ Coordinate [Real Robot]:
------------------------------------------------------------
Initial Pose to Final Pose in Single Step Movement: [be extra careful when using this command]
ros2 launch ur3e_helper_scripts move_to_coords.launch.py x:=0.250 y:=0.250 z:=0.250 steps:=once
Initial Pose to Final Pose in Triple Step Movement: [steps can be any valid combination of x, y, z]
ros2 launch ur3e_helper_scripts move_to_coords.launch.py x:=0.250 y:=0.250 z:=0.250 steps:=zyx

For Block [Real Robot]:
-----------------------
Real Robot, Block, Standing, Slim Grip:
ros2 launch ur3e_helper_scripts setup_grasp_object.launch.py use_block:=True standing:=True slim_grip:=True

Real Robot, Block, Standing, Wide Grip:
ros2 launch ur3e_helper_scripts setup_grasp_object.launch.py use_block:=True standing:=True wide_grip:=True

Real Robot, Block, Lying, Slim Grip:
ros2 launch ur3e_helper_scripts setup_grasp_object.launch.py use_block:=True lying:=True slim_grip:=True

Real Robot, Block, Lying, Wide Grip:
ros2 launch ur3e_helper_scripts setup_grasp_object.launch.py use_block:=True lying:=True wide_grip:=True

For Cube [Real Robot]:
----------------------
ros2 launch ur3e_helper_scripts setup_grasp_object.launch.py use_cube:=True

Reset Grasp Object from Pick Position to a given XYZ Coordinate [Real Robot]:
-----------------------------------------------------------------------------
ros2 launch ur3e_helper_scripts reset_grasp_object.launch.py x:=0.150 y:=0.250 z:=0.200


Other Useful Commands:
----------------------

Gripper Control [Simulation]:
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "command: {position: 0.0, max_effort: 0.0}"
Gripper Control [Real Robot]:
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "command: {position: 0.0, max_effort: 0.0}"

Get End Effector Pose: [with respect to base_link (same as world)]
ros2 run tf2_ros tf2_echo base_link tool0

Display UR3 Arm Joint Values as Radians [Real Robot]:
ros2 topic echo /joint_states --once
Display Gripper Joint Values as Radians [Real Robot]:
ros2 topic echo /gripper/joint_states --once


Some Extra Words:
-----------------
This program does not use Inverse Kinematics for Path Planning. This is because, in the simulation, the objects keep falling off the gripper.
On an average, 2 out of 10 times, the gripper grips the object correctly and takes it to the target position, with Cartesian Path Planning.
When Inverse Kinematics is used, the object does not get gripped properly every time and simulation fails ~90% of the times.
Inverse Kinematics makes weird path plans that also sometimes collides into nearby solid objects like table, wall, etc.
Therefore, the Path Planning ONLY uses fixed Cartesian Movements with Waypoints to target poses.


License:
--------
This package and its contents were developed using ROS2 and MoveIt2 open-source framework(s). This package contents are open to all.
Originally designed for students who have enrolled in the MasterClass of The Construct which offers Robotics Training Programs.
Contents are designed and maintained by Girish Kumar Kannan (email: girishkumar.kannan@gmail.com).


# ~~~~~~~~~~ The End ~~~~~~~~~~ #
