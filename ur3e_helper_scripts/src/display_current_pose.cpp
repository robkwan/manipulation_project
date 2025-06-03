#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <chrono>
#include <memory>
#include <thread>

// static variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");
static const std::string PLANNING_GROUP_UR3_ARM = "ur3_arm";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

int main(int argc, char **argv) {
  // initialize program node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  // auto-declare node_options parameters from overrides
  node_options.automatically_declare_parameters_from_overrides(true);

  // initialize move_group node
  rclcpp::Node::SharedPtr move_group_node =
      rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // start move_group node in a new executor thread and spin it
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // initialize move_group_interface for ur3_arm and gripper
  moveit::planning_interface::MoveGroupInterface move_group_ur3_arm(
      move_group_node, PLANNING_GROUP_UR3_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  // get initial state of ur3_arm and gripper
  const moveit::core::JointModelGroup *joint_model_group_ur3_arm =
      move_group_ur3_arm.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_UR3_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

  // initialize trajectory planning variables for ur3_arm and gripper
  std::vector<double> joint_group_positions_ur3_arm;
  moveit::core::RobotStatePtr current_state_ur3_arm;
  geometry_msgs::msg::Pose current_pose_ur3_arm;
  std::vector<double> joint_group_positions_gripper;
  moveit::core::RobotStatePtr current_state_gripper;
  geometry_msgs::msg::Pose current_pose_gripper;

  // get current state of ur3_arm and gripper
  current_state_ur3_arm = move_group_ur3_arm.getCurrentState(10);
  current_state_ur3_arm->copyJointGroupPositions(joint_model_group_ur3_arm,
                                                 joint_group_positions_ur3_arm);
  current_state_gripper = move_group_gripper.getCurrentState(10);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

  // set start state of ur3_arm and gripper to current state
  move_group_ur3_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();

  // get the current state of ur3 arm and gripper
  current_pose_ur3_arm = move_group_ur3_arm.getCurrentPose().pose;
  current_pose_gripper = move_group_gripper.getCurrentPose().pose;

  // print current pose of ur3 arm and gripper
  RCLCPP_INFO(
      LOGGER,
      "\n Current Pose UR3 Arm: Px:%+0.4f Py:%+0.4f Pz:%+0.4f Qx:%+0.4f "
      "Qy:%+0.4f "
      "Qz:%+0.4f Qw:%+0.4f",
      current_pose_ur3_arm.position.x, current_pose_ur3_arm.position.y,
      current_pose_ur3_arm.position.z, current_pose_ur3_arm.orientation.x,
      current_pose_ur3_arm.orientation.y, current_pose_ur3_arm.orientation.z,
      current_pose_ur3_arm.orientation.w);
  RCLCPP_INFO(
      LOGGER,
      "\n Current Pose Gripper: Px:%+0.4f Py:%+0.4f Pz:%+0.4f Qx:%+0.4f "
      "Qy:%+0.4f "
      "Qz:%+0.4f Qw:%+0.4f",
      current_pose_gripper.position.x, current_pose_gripper.position.y,
      current_pose_gripper.position.z, current_pose_gripper.orientation.x,
      current_pose_gripper.orientation.y, current_pose_gripper.orientation.z,
      current_pose_gripper.orientation.w);

  // shutdown program node
  rclcpp::shutdown();
  return 0;
}

// End of Code
