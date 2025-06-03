#include "moveit_msgs/msg/detail/collision_object__struct.hpp"
#include "shape_msgs/msg/detail/solid_primitive__struct.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

// session variables
// // simulation - grasp object dimensions in meters
static const float sim_block_l = +0.040; // grasp block length
static const float sim_block_b = +0.020; // grasp block breadth
static const float sim_block_h = +0.080; // grasp block height
static const float sim_cube_s = +0.050;  // grasp cube side
// // simulation - grasp object target position in meters
static const float sim_pos_x = +0.340; // adjust as required
static const float sim_pos_y = -0.020; // adjust as required
// // real robot - grasp object dimensions in meters
static const float real_block_l = +0.025; // grasp block length
static const float real_block_b = +0.015; // grasp block breadth
static const float real_block_h = +0.075; // grasp block height
static const float real_cube_s = +0.050;  // grasp cube side
// // real robot - grasp object target position in meters
static const float real_pos_x = +0.343; // adjust as required
static const float real_pos_y = +0.132; // adjust as required
// // grasp object placement height tolerance in meters
static const float tolerance = +0.005; // adjust as required

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP_UR3_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class SetupGraspObject {

public:
  SetupGraspObject(rclcpp::Node::SharedPtr base_node) : base_node(base_node) {
    // configure node options
    rclcpp::NodeOptions node_options;
    // auto-declare node_options parameters from overrides
    node_options.automatically_declare_parameters_from_overrides(true);

    // declare launch parameters
    // Note: "use_sim_time" is already declared!
    base_node->declare_parameter("use_cube", false);
    base_node->declare_parameter("use_block", true);
    base_node->declare_parameter("lying", false);
    base_node->declare_parameter("standing", true);
    base_node->declare_parameter("slim_grip", false);
    base_node->declare_parameter("wide_grip", true);

    // initialize move_group node
    move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial",
                                                node_options);
    // start move_group node in a new executor thread and spin it
    executor.add_node(move_group_node);
    std::thread([this]() { this->executor.spin(); }).detach();

    // initialize move_group interfaces
    move_group_ur3_arm = std::make_shared<MoveGroupInterface>(
        move_group_node, PLANNING_GROUP_UR3_ARM);
    move_group_gripper = std::make_shared<MoveGroupInterface>(
        move_group_node, PLANNING_GROUP_GRIPPER);

    // get initial state of ur3_arm and gripper
    joint_model_group_ur3_arm =
        move_group_ur3_arm->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_UR3_ARM);
    joint_model_group_gripper =
        move_group_gripper->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    // get current states of ur3_arm and gripper
    get_current_state_ur3_arm();
    get_current_state_gripper();

    // set start state of ur3_arm and gripper to current state
    move_group_ur3_arm->setStartStateToCurrentState();
    move_group_gripper->setStartStateToCurrentState();

    // get launch parameters
    use_sim_time = base_node->get_parameter("use_sim_time").as_bool();
    use_cube = base_node->get_parameter("use_cube").as_bool();
    use_block = base_node->get_parameter("use_block").as_bool();
    lying = base_node->get_parameter("lying").as_bool();
    standing = base_node->get_parameter("standing").as_bool();
    slim_grip = base_node->get_parameter("slim_grip").as_bool();
    wide_grip = base_node->get_parameter("wide_grip").as_bool();

    // decide if object is cube or block (cuboid)
    // flag is conditioned on use_cube
    if (!use_cube) {
      RCLCPP_INFO(LOGGER, "Grasp Object: BLOCK (Not CUBE)");
      use_block = true;
    } else {
      RCLCPP_INFO(LOGGER, "Grasp Object: CUBE (Not BLOCK)");
      use_block = false;
      lying = false;    // cube is always standing!
      wide_grip = true; // cube is always wide grip!
    }
    // decide if object is standing or lying
    // flag is conditioned on lying
    if (!lying) {
      RCLCPP_INFO(LOGGER, "Grasp Pose: STANDING (Not LYING)");
      standing = true;
    } else {
      RCLCPP_INFO(LOGGER, "Grasp Pose: LYING (Not STANDING)");
      standing = false;
    }
    // decide to use wide grip or slim grip
    // flag is conditioned on wide_grip
    if (!wide_grip) {
      RCLCPP_INFO(LOGGER, "Grasp Grip: SLIM (Not WIDE)");
      slim_grip = true;
    } else {
      RCLCPP_INFO(LOGGER, "Grasp Grip: WIDE (Not SLIM)");
      slim_grip = false;
    }

    // choose variables based on simulation or real robot
    if (use_sim_time) {
      if (use_block) {
        object_l = sim_block_l;
        object_b = sim_block_b;
        object_h = sim_block_h;
      } else {
        object_l = sim_cube_s;
        object_b = sim_cube_s;
        object_h = sim_cube_s;
      }
      object_x = sim_pos_x;
      object_y = sim_pos_y;
    } else {
      if (use_block) {
        object_l = real_block_l;
        object_b = real_block_b;
        object_h = real_block_h;
      } else {
        object_l = real_cube_s;
        object_b = real_cube_s;
        object_h = real_cube_s;
      }
      object_x = real_pos_x;
      object_y = real_pos_y;
    }
    if (slim_grip) {
      gripper_value = calc_gripper_value(object_b);
    } else {
      gripper_value = calc_gripper_value(object_l);
    }

    // add collision objects to planning scene
    add_planning_scene_objects();
  }

  ~SetupGraspObject() {}

  void execute_trajectory_plan() {
    if (standing) {
      // grasp object is standing
      execute_trajectory_standing();
    } else {
      // grasp object is lying
      execute_trajectory_lying();
    }
  }

  void execute_trajectory_lying() {
    // ~~~~~~~~~~ start trajectory planning ~~~~~~~~~~ //

    // print gripper value
    RCLCPP_INFO(LOGGER, "Gripper Value: %0.4f", gripper_value);
    sleep_ms(1000);

    // at this point - gripper is open - ready to grip the block
    get_current_pose_ur3_arm();
    float grasp_grip_height = current_pose_ur3_arm.position.z;
    RCLCPP_INFO(LOGGER, "Grasp Grip Height: %0.4f", grasp_grip_height);
    update_debug_vector("Blocks Pick Pose - Gripper Opened");

    // close gripper carefully
    RCLCPP_INFO(LOGGER, "Closing the Gripper");
    close_gripper(gripper_value);
    update_debug_vector("Blocks Pick Pose - Gripper Closed");

    // retreat from blocks pick position
    RCLCPP_INFO(LOGGER, "Retreat from Blocks Pick Position");
    plan_and_execute_waypoints_target(+0.350, "z");
    update_debug_vector("Blocks Pick Pose - Post-Retreat");

    // go to in-transit pose 1
    RCLCPP_INFO(LOGGER, "Going to In-Transit Position 1");
    plan_and_execute_joint_value_target(+0.0000, -1.5708, +1.5708, -1.5708,
                                        -1.5708, +0.0000);
    update_debug_vector("In-Transit Pose 1");

    // go to in-transit pose 2
    RCLCPP_INFO(LOGGER, "Going to In-Transit Position 2");
    plan_and_execute_joint_value_target(+0.0000, -1.5708, +1.5708, +0.0000,
                                        +0.0000, +0.0000);
    update_debug_vector("In-Transit Pose 2");

    // go to in-transit place position along x axis
    RCLCPP_INFO(LOGGER, "Going to In-Transit Place Position along X-Axis");
    plan_and_execute_waypoints_target(object_x, "x"); // -0.020 offset
    update_debug_vector("In-Transit Place Pose - Post-X-Travel");

    // go to in-transit place position along y axis
    RCLCPP_INFO(LOGGER, "Going to In-Transit Place Position along Y-Axis");
    float pos_y = 0.0;
    if (wide_grip) {
      pos_y = (object_y - grasp_grip_height - (object_b / 2.0));
    } else {
      pos_y = (object_y - 0.163 - (object_l / 2.0)); // gripper length closed
    }
    plan_and_execute_waypoints_target(pos_y, "y");
    update_debug_vector("In-Transit Place Pose - Post-Y-Travel");

    // approach to in-transit place position
    RCLCPP_INFO(LOGGER, "Approach to In-Transit Place Position");
    plan_and_execute_waypoints_target(+0.060, "z");
    update_debug_vector("In-Transit Place Pose - Post-Approach");

    // open gripper carefully
    RCLCPP_INFO(LOGGER, "Opening the Gripper");
    open_gripper();
    update_debug_vector("In-Transit Place Pose - Gripper Opened");

    // retreat from in-transit place position
    RCLCPP_INFO(LOGGER, "Retreat from In-Transit Place Position");
    plan_and_execute_waypoints_target(+0.200, "z");
    update_debug_vector("In-Transit Place Pose - Post-Retreat");

    // go to in-transit pose 1
    RCLCPP_INFO(LOGGER, "Going to In-Transit Position 1");
    if (wide_grip) {
      plan_and_execute_joint_value_target(+0.0000, -1.5708, +1.5708, -1.5708,
                                          -1.5708, +0.0000);
    } else {
      plan_and_execute_joint_value_target(+0.0000, -1.5708, +1.5708, -1.5708,
                                          -1.5708, -1.5708);
    }
    update_debug_vector("In-Transit Pose 1");

    // go to in-transit pick position along x axis
    RCLCPP_INFO(LOGGER, "Going to In-Transit Pick Position along X-Axis");
    plan_and_execute_waypoints_target(object_x, "x");
    update_debug_vector("In-Transit Pick Pose - Post-X-Travel");

    // go to in-transit pick position along y axis
    RCLCPP_INFO(LOGGER, "Going to In-Transit Pick Position along Y-Axis");
    plan_and_execute_waypoints_target(object_y, "y");
    update_debug_vector("In-Transit Pick Pose - Post-Y-Travel");

    // approach to in-transit pick position
    RCLCPP_INFO(LOGGER, "Approach to In-Transit Pick Position");
    plan_and_execute_waypoints_target(+0.200, "z");
    update_debug_vector("In-Transit Pick Pose - Post-Approach");

    // close gripper carefully
    RCLCPP_INFO(LOGGER, "Closing the Gripper");
    gripper_value = calc_gripper_value(object_b); // always breadth!
    close_gripper(gripper_value);
    update_debug_vector("In-Transit Pick Pose - Gripper Closed");

    // retreat from in-transit pick position
    RCLCPP_INFO(LOGGER, "Retreat from In-Transit Pick Position");
    plan_and_execute_waypoints_target(+0.250, "z");
    update_debug_vector("In-Transit Pick Pose - Post-Retreat");

    // go to place position along x axis
    RCLCPP_INFO(LOGGER, "Going to Place Position along X-Axis");
    plan_and_execute_waypoints_target(object_x, "x");
    update_debug_vector("Place Pose - Post-X-Travel");

    // go to place position along y axis
    RCLCPP_INFO(LOGGER, "Going to Place Position along Y-Axis");
    plan_and_execute_waypoints_target(object_y, "y");
    update_debug_vector("Place Pose - Post-Y-Travel");

    // set end-effector link orientation
    RCLCPP_INFO(LOGGER, "Setting End-Effector Orientation");
    get_current_state_ur3_arm();
    // get wrist_3_joint angle with respect to shoulder_pan_joint
    float end_effector_orientation = 0.0;
    if (wide_grip) {
      end_effector_orientation = joint_group_positions_ur3_arm[0];
      end_effector_orientation += (float)(-M_PI_2);
    } else {
      end_effector_orientation = joint_group_positions_ur3_arm[5];
    }
    // set wrist_3_joint angle to calculated end_effector_orientation
    joint_group_positions_ur3_arm[5] = end_effector_orientation;
    move_group_ur3_arm->setJointValueTarget(joint_group_positions_ur3_arm);
    plan_and_execute_ur3_arm_kinematic();
    update_debug_vector("Place Pose - Post-Orientation");

    // approach the place position
    RCLCPP_INFO(LOGGER, "Approach to Place Position");
    // add height tolerance at place position
    plan_and_execute_waypoints_target((+0.200 + tolerance), "z");
    update_debug_vector("Place Pose - Post-Approach");

    // open gripper carefully
    RCLCPP_INFO(LOGGER, "Opening the Gripper");
    open_gripper();
    update_debug_vector("Place Pose - Gripper Opened");

    // retreat from place position
    RCLCPP_INFO(LOGGER, "Retreat from Place Position");
    plan_and_execute_waypoints_target(+0.350, "z");
    update_debug_vector("Place Pose - Post-Retreat");

    // go to vertical home position
    RCLCPP_INFO(LOGGER, "Going to Vertical Home Position");
    plan_and_execute_joint_value_target(+0.0000, -1.5708, +0.0000, -1.5708,
                                        +0.0000, -1.5708);
    update_debug_vector("Vertical Home Pose");

    // print debug poses
    print_debug_vector();

    // ~~~~~~~~~~ finish trajectory planning ~~~~~~~~~~ //
  }

  void execute_trajectory_standing() {
    // ~~~~~~~~~~ start trajectory planning ~~~~~~~~~~ //

    // print gripper value
    RCLCPP_INFO(LOGGER, "Gripper Value: %0.4f", gripper_value);
    sleep_ms(1000);

    // at this point - gripper is open - ready to grip the block
    get_current_pose_ur3_arm();
    float grasp_grip_height = current_pose_ur3_arm.position.z;
    RCLCPP_INFO(LOGGER, "Grasp Grip Height: %0.4f", grasp_grip_height);
    update_debug_vector("Blocks Pick Pose - Gripper Opened");

    // close gripper carefully
    RCLCPP_INFO(LOGGER, "Closing the Gripper");
    close_gripper(gripper_value);
    update_debug_vector("Blocks Pick Pose - Gripper Closed");

    // retreat from blocks pick position
    RCLCPP_INFO(LOGGER, "Retreat from Blocks Pick Position");
    plan_and_execute_waypoints_target(+0.350, "z");
    update_debug_vector("Blocks Pick Pose - Post-Retreat");

    // go to in-transit position
    RCLCPP_INFO(LOGGER, "Going to In-Transit Position");
    plan_and_execute_joint_value_target(+0.7854, -1.5708, +1.5708, -1.5708,
                                        -1.5708, +0.0000);
    update_debug_vector("In-Transit Pose");

    // go to place position along x axis
    RCLCPP_INFO(LOGGER, "Going to Place Position along X-Axis");
    plan_and_execute_waypoints_target(object_x, "x");
    update_debug_vector("Place Pose - Post-X-Travel");

    // go to place position along y axis
    RCLCPP_INFO(LOGGER, "Going to Place Position along Y-Axis");
    plan_and_execute_waypoints_target(object_y, "y");
    update_debug_vector("Place Pose - Post-Y-Travel");

    // set end-effector link orientation
    RCLCPP_INFO(LOGGER, "Setting End-Effector Orientation");
    get_current_state_ur3_arm();
    // get wrist_3_joint angle with respect to shoulder_pan_joint
    float end_effector_orientation = joint_group_positions_ur3_arm[0];
    if (slim_grip) {
      end_effector_orientation += (float)(-M_PI_2);
    } else {
      // no changes here
    }
    // set wrist_3_joint angle to calculated end_effector_orientation
    joint_group_positions_ur3_arm[5] = end_effector_orientation;
    move_group_ur3_arm->setJointValueTarget(joint_group_positions_ur3_arm);
    plan_and_execute_ur3_arm_kinematic();
    update_debug_vector("Place Pose - Post-Orientation");

    // approach the place position
    RCLCPP_INFO(LOGGER, "Approach to Place Position");
    // add height tolerance at place position
    float place_height = (grasp_grip_height + tolerance);
    if (slim_grip) {
      place_height += 0.001; // add 1mm more to z-axis
    } else {
      // no changes here
    }
    plan_and_execute_waypoints_target(place_height, "z");
    update_debug_vector("Place Pose - Post-Approach");

    // open gripper carefully
    RCLCPP_INFO(LOGGER, "Opening the Gripper");
    open_gripper();
    update_debug_vector("Place Pose - Gripper Opened");

    // retreat from place position
    RCLCPP_INFO(LOGGER, "Retreat from Place Position");
    plan_and_execute_waypoints_target(+0.350, "z");
    update_debug_vector("Place Pose - Post-Retreat");

    // go to vertical home position
    RCLCPP_INFO(LOGGER, "Going to Vertical Home Position");
    plan_and_execute_joint_value_target(+0.0000, -1.5708, +0.0000, -1.5708,
                                        +0.0000, -1.5708);
    update_debug_vector("Vertical Home Pose");

    // print debug poses
    print_debug_vector();

    // ~~~~~~~~~~ finish trajectory planning ~~~~~~~~~~ //
  }

private:
  // using shorthand for lengthy class references
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using JointModelGroup = moveit::core::JointModelGroup;
  using RobotStatePtr = moveit::core::RobotStatePtr;
  using Plan = MoveGroupInterface::Plan;
  using Pose = geometry_msgs::msg::Pose;
  using RobotTrajectory = moveit_msgs::msg::RobotTrajectory;
  using CollisionObject = moveit_msgs::msg::CollisionObject;
  using SolidPrimitive = shape_msgs::msg::SolidPrimitive;
  using PlanningSceneInterface =
      moveit::planning_interface::PlanningSceneInterface;

  // declare rclcpp base node class
  rclcpp::Node::SharedPtr base_node;

  // declare move_group_node
  rclcpp::Node::SharedPtr move_group_node;

  // declare single threaded executor for move_group node
  rclcpp::executors::SingleThreadedExecutor executor;

  // declare planning scene collision object variable
  CollisionObject collision_object;
  SolidPrimitive ur3e_desk, ur3e_wall;
  Pose ur3e_desk_pose, ur3e_wall_pose;
  PlanningSceneInterface planning_scene_interface;

  // declare move_group_interface varibles for ur3_arm and gripper
  std::shared_ptr<MoveGroupInterface> move_group_ur3_arm;
  std::shared_ptr<MoveGroupInterface> move_group_gripper;

  // declare joint_model_group for ur3_arm and gripper
  const JointModelGroup *joint_model_group_ur3_arm;
  const JointModelGroup *joint_model_group_gripper;

  // declare trajectory planning variables for ur3_arm and gripper
  std::vector<double> joint_group_positions_ur3_arm;
  RobotStatePtr current_state_ur3_arm;
  Plan trajectory_plan_ur3_arm;
  Pose current_pose_ur3_arm;
  Pose target_pose_ur3_arm;
  bool plan_success_ur3_arm = false;
  std::vector<double> joint_group_positions_gripper;
  RobotStatePtr current_state_gripper;
  Plan trajectory_plan_gripper;
  bool plan_success_gripper = false;
  const float gripper_minimum = +0.000;   // radians
  const float gripper_maximum = +0.800;   // radians
  const float gripper_tolerance = +0.035; // radians
  float gripper_value = 0.0;

  // initialize common cartesian path planning variables
  double fraction = 0.0;
  const double jump_threshold = 0.0;
  const double end_effector_step = 0.01;
  std::vector<Pose> cartesian_waypoints;
  RobotTrajectory cartesian_trajectory;

  // initialize debug print variables
  std::vector<std::string> pose_names;
  std::vector<Pose> pose_values;

  // declare launch variables as class variables
  bool use_sim_time;
  bool use_cube, use_block;
  bool standing, lying;
  bool wide_grip, slim_grip;

  // initialize dimension variables
  float object_l = 0.0, object_b = 0.0, object_h = 0.0;

  // initialize coordinate variables
  float object_x = 0.0, object_y = 0.0;

  void add_planning_scene_objects() {
    collision_object.header.frame_id = "base_link";
    collision_object.id = "ur3e_warehouse_collisions";

    ur3e_desk.type = ur3e_desk.BOX;
    ur3e_desk.dimensions.resize(3);
    ur3e_desk.dimensions[ur3e_desk.BOX_X] = 1.8288;
    ur3e_desk.dimensions[ur3e_desk.BOX_Y] = 0.6096;
    ur3e_desk.dimensions[ur3e_desk.BOX_Z] = 0.0508;
    ur3e_desk_pose.position.x = +0.7739;
    ur3e_desk_pose.position.y = +0.1817;
    ur3e_desk_pose.position.z = -0.0254;
    ur3e_desk_pose.orientation.x = 0.0000;
    ur3e_desk_pose.orientation.y = 0.0000;
    ur3e_desk_pose.orientation.z = 0.0000;
    ur3e_desk_pose.orientation.w = 1.0000;
    collision_object.primitives.push_back(ur3e_desk);
    collision_object.primitive_poses.push_back(ur3e_desk_pose);

    ur3e_wall.type = ur3e_wall.BOX;
    ur3e_wall.dimensions.resize(3);
    ur3e_wall.dimensions[ur3e_wall.BOX_X] = 2.4384;
    ur3e_wall.dimensions[ur3e_wall.BOX_Y] = 0.1524;
    ur3e_wall.dimensions[ur3e_wall.BOX_Z] = 2.4384;
    ur3e_wall_pose.position.x = +0.4691;
    ur3e_wall_pose.position.y = +0.5627;
    ur3e_wall_pose.position.z = +0.2992;
    ur3e_wall_pose.orientation.x = 0.0000;
    ur3e_wall_pose.orientation.y = 0.0000;
    ur3e_wall_pose.orientation.z = 0.0000;
    ur3e_wall_pose.orientation.w = 1.0000;
    collision_object.primitives.push_back(ur3e_wall);
    collision_object.primitive_poses.push_back(ur3e_wall_pose);

    collision_object.operation = collision_object.ADD;
    planning_scene_interface.applyCollisionObject(collision_object);
  }

  void get_current_state_ur3_arm() {
    // get current state of ur3_arm
    current_state_ur3_arm = move_group_ur3_arm->getCurrentState(10);
    current_state_ur3_arm->copyJointGroupPositions(
        joint_model_group_ur3_arm, joint_group_positions_ur3_arm);
  }

  void get_current_state_gripper() {
    // get current state of gripper
    current_state_gripper = move_group_gripper->getCurrentState(10);
    current_state_gripper->copyJointGroupPositions(
        joint_model_group_gripper, joint_group_positions_gripper);
  }

  void get_current_pose_ur3_arm() {
    current_pose_ur3_arm = move_group_ur3_arm->getCurrentPose().pose;
  }

  void set_target_pose_to_current_pose_ur3_arm() {
    target_pose_ur3_arm = move_group_ur3_arm->getCurrentPose().pose;
  }

  void sleep_ms(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  }

  void plan_and_execute_ur3_arm_kinematic() {
    plan_success_ur3_arm = (move_group_ur3_arm->plan(trajectory_plan_ur3_arm) ==
                            moveit::core::MoveItErrorCode::SUCCESS);
    if (plan_success_ur3_arm) {
      move_group_ur3_arm->execute(trajectory_plan_ur3_arm);
      RCLCPP_INFO(LOGGER, "UR3 Arm Kinematic Action Success !");
    } else {
      RCLCPP_INFO(LOGGER, "UR3 Arm Kinematic Action Failed !");
    }
  }

  void plan_and_execute_ur3_arm_cartesian() {
    fraction = move_group_ur3_arm->computeCartesianPath(
        cartesian_waypoints, end_effector_step, jump_threshold,
        cartesian_trajectory);
    if (fraction >= 0.0) {
      move_group_ur3_arm->execute(cartesian_trajectory);
      RCLCPP_INFO(LOGGER, "UR3 Arm Cartesian Action Success !");
    } else {
      RCLCPP_INFO(LOGGER, "UR3 Arm Cartesian Action Failed !");
    }
    // clear waypoints
    cartesian_waypoints.clear();
  }

  void plan_and_execute_joint_value_target(float angle0, float angle1,
                                           float angle2, float angle3,
                                           float angle4, float angle5) {
    joint_group_positions_ur3_arm[0] = angle0; // Shoulder Pan
    joint_group_positions_ur3_arm[1] = angle1; // Shoulder Lift
    joint_group_positions_ur3_arm[2] = angle2; // Elbow
    joint_group_positions_ur3_arm[3] = angle3; // Wrist 1
    joint_group_positions_ur3_arm[4] = angle4; // Wrist 2
    joint_group_positions_ur3_arm[5] = angle5; // Wrist 3
    move_group_ur3_arm->setJointValueTarget(joint_group_positions_ur3_arm);
    plan_and_execute_ur3_arm_kinematic();
  }

  void plan_and_execute_end_effector_target(float pos_x, float pos_y,
                                            float pos_z, float quat_x,
                                            float quat_y, float quat_z,
                                            float quat_w) {
    target_pose_ur3_arm.position.x = pos_x;
    target_pose_ur3_arm.position.y = pos_y;
    target_pose_ur3_arm.position.z = pos_z;
    target_pose_ur3_arm.orientation.x = quat_x;
    target_pose_ur3_arm.orientation.y = quat_y;
    target_pose_ur3_arm.orientation.z = quat_z;
    target_pose_ur3_arm.orientation.w = quat_w;
    move_group_ur3_arm->setPoseTarget(target_pose_ur3_arm);
    plan_and_execute_ur3_arm_kinematic();
  }

  void plan_and_execute_waypoints_target(float target_value, std::string axis) {
    set_target_pose_to_current_pose_ur3_arm();
    cartesian_waypoints.push_back(target_pose_ur3_arm);
    if (axis == "z") {
      target_pose_ur3_arm.position.z = target_value;
    } else if (axis == "y") {
      target_pose_ur3_arm.position.y = target_value;
    } else if (axis == "x") {
      target_pose_ur3_arm.position.x = target_value;
    } else {
      RCLCPP_INFO(LOGGER, "Invalid Waypoints Target !");
    }
    cartesian_waypoints.push_back(target_pose_ur3_arm);
    plan_and_execute_ur3_arm_cartesian();
  }

  void plan_and_execute_gripper() {
    plan_success_gripper = (move_group_gripper->plan(trajectory_plan_gripper) ==
                            moveit::core::MoveItErrorCode::SUCCESS);
    if (plan_success_gripper) {
      move_group_gripper->execute(trajectory_plan_gripper);
      RCLCPP_INFO(LOGGER, "Gripper Action Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Gripper Action Failed !");
    }
  }

  void gripper_action(float value) {
    // set gripper range limits
    if (value < gripper_minimum) {
      joint_group_positions_gripper[2] = gripper_minimum;
    } else if (value > gripper_maximum) {
      joint_group_positions_gripper[2] = gripper_maximum;
    } else {
      joint_group_positions_gripper[2] = value;
    }
    RCLCPP_INFO(LOGGER, "Setting Gripper Value: %+0.4f",
                joint_group_positions_gripper[2]);
    move_group_gripper->setJointValueTarget(joint_group_positions_gripper);
    plan_and_execute_gripper();
    sleep_ms(100);
  }

  void open_gripper() {
    // multi stage gripper open action for safe releasing of object
    get_current_state_gripper();
    float value = joint_group_positions_gripper[2];
    for (char i = 1; i <= 4; i++) {
      // decrease the value
      value -= (gripper_tolerance * (float)(i));
      // then do gripper action
      gripper_action(value);
    }
    gripper_action(gripper_minimum);
    get_current_state_gripper();
    sleep_ms(500);
  }

  void close_gripper(float value) {
    // multi stage gripper close action for safe gripping of object
    get_current_state_gripper();
    value -= (6.0 * gripper_tolerance);
    for (char i = 1; i <= 4; i++) {
      // do gripper action
      gripper_action(value);
      // then increase the value
      value += (gripper_tolerance * (float)(4 - i));
    }
    if (!use_sim_time) {
      gripper_action(gripper_maximum);
    } else {
      // do nothing!
    }
    get_current_state_gripper();
    sleep_ms(500);
  }

  void update_debug_vector(std::string pose_name) {
    pose_names.push_back(pose_name);
    pose_values.push_back(move_group_ur3_arm->getCurrentPose().pose);
    sleep_ms(1000);
  }

  void print_debug_vector() {
    for (unsigned long pose = 0; pose < pose_names.size(); pose++) {
      RCLCPP_INFO(
          LOGGER,
          "\n %s : Px:%+0.4f Py:%+0.4f Pz:%+0.4f "
          "Qx:%+0.4f Qy:%+0.4f Qz:%+0.4f Qw:%+0.4f",
          pose_names[pose].c_str(), pose_values[pose].position.x,
          pose_values[pose].position.y, pose_values[pose].position.z,
          pose_values[pose].orientation.x, pose_values[pose].orientation.y,
          pose_values[pose].orientation.z, pose_values[pose].orientation.w);
    }
  }

  float calc_gripper_value(float distance) {
    float value = 0.0;
    if (!use_sim_time) {
      // real robot gripper function
      value =
          ((-30.8904) * powf(distance, 3.0) + (-1.5857) * powf(distance, 2.0) +
           (-8.1745) * powf(distance, 1.0) + (+0.7826));
    } else {
      // simulation gripper function
      value =
          ((-205.7368) * powf(distance, 3.0) + (+8.2693) * powf(distance, 2.0) +
           (-8.6497) * powf(distance, 1.0) + (+0.8033));
    }
    return value;
  }

}; // class SetupGraspObject

int main(int argc, char **argv) {
  // initialize program node
  rclcpp::init(argc, argv);

  // initialize nase_node as shared pointer
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("setup_grasp_object");

  // instantiate class
  SetupGraspObject setup_grasp_object_node(base_node);

  // execute trajectory plan
  setup_grasp_object_node.execute_trajectory_plan();

  // shutdown ros2 node
  rclcpp::shutdown();

  return 0;
}

// End of Code
