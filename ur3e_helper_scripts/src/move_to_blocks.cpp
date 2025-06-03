#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP_UR3_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class MoveToBlocks {

public:
  MoveToBlocks(rclcpp::Node::SharedPtr base_node) : base_node(base_node) {
    // configure node options
    rclcpp::NodeOptions node_options;
    // auto-declare node_options parameters from overrides
    node_options.automatically_declare_parameters_from_overrides(true);

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
  }

  ~MoveToBlocks() {}

  void execute_trajectory_plan() {
    // ~~~~~~~~~~ start trajectory planning ~~~~~~~~~~ //

    // go to vertical home position
    RCLCPP_INFO(LOGGER, "Going to Vertical Home Position");
    plan_and_execute_joint_value_target(+0.0000, -1.5708, +0.0000, -1.5708,
                                        +0.0000, -1.5708);
    update_debug_vector("Vertical Home Pose");

    // open gripper carefully
    RCLCPP_INFO(LOGGER, "Opening the Gripper");
    gripper_action(gripper_minimum);
    update_debug_vector("Vertical Home Pose - Gripper Opened");

    // go to blocks pick pose
    RCLCPP_INFO(LOGGER, "Going to Blocks Pick Position");
    plan_and_execute_joint_value_target(+1.5708, -1.5708, +1.5708, -1.5708,
                                        -1.5708, +0.0000);
    update_debug_vector("Blocks Pick Pose");

    // approach to blocks pick position
    RCLCPP_INFO(LOGGER, "Approach to Blocks Pick Position");
    plan_and_execute_waypoints_target(+0.250, "z");
    update_debug_vector("Blocks Pick Pose - Post-Approach");

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

  // declare rclcpp base node class
  rclcpp::Node::SharedPtr base_node;

  // declare move_group_node
  rclcpp::Node::SharedPtr move_group_node;

  // declare single threaded executor for move_group node
  rclcpp::executors::SingleThreadedExecutor executor;

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

  // initialize common cartesian path planning variables
  double fraction = 0.0;
  const double jump_threshold = 0.0;
  const double end_effector_step = 0.01;
  std::vector<Pose> cartesian_waypoints;
  RobotTrajectory cartesian_trajectory;

  // initialize debug print variables
  std::vector<std::string> pose_names;
  std::vector<Pose> pose_values;

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
    move_group_gripper->setJointValueTarget(joint_group_positions_gripper);
    plan_and_execute_gripper();
    sleep_ms(100);
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

}; // class MoveToBlocks

int main(int argc, char **argv) {
  // initialize program node
  rclcpp::init(argc, argv);

  // initialize nase_node as shared pointer
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("move_to_blocks");

  // instantiate class
  MoveToBlocks move_to_blocks_node(base_node);

  // execute trajectory plan
  move_to_blocks_node.execute_trajectory_plan();

  // shutdown ros2 node
  rclcpp::shutdown();

  return 0;
}

// End of Code
