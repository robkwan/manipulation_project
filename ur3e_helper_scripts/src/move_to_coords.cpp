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
static const std::string PLANNING_GROUP_UR3_ARM = "ur3_arm";

class MoveToCoords {

public:
  MoveToCoords(rclcpp::Node::SharedPtr base_node) : base_node(base_node) {
    // configure node options
    rclcpp::NodeOptions node_options;
    // auto-declare node_options parameters from overrides
    node_options.automatically_declare_parameters_from_overrides(true);

    // declare launch parameters
    // Note: "use_sim_time" is already declared!
    base_node->declare_parameter("x", +0.250);
    base_node->declare_parameter("y", +0.250);
    base_node->declare_parameter("z", +0.250);
    base_node->declare_parameter("steps", "once");

    // initialize move_group node
    move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial",
                                                node_options);
    // start move_group node in a new executor thread and spin it
    executor.add_node(move_group_node);
    std::thread([this]() { this->executor.spin(); }).detach();

    // initialize move_group interfaces
    move_group_ur3_arm = std::make_shared<MoveGroupInterface>(
        move_group_node, PLANNING_GROUP_UR3_ARM);

    // get initial state of ur3_arm and gripper
    joint_model_group_ur3_arm =
        move_group_ur3_arm->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_UR3_ARM);

    // get current states of ur3_arm
    get_current_state_ur3_arm();

    // set start state of ur3_arm to current state
    move_group_ur3_arm->setStartStateToCurrentState();

    // get launch parameters
    use_sim_time = base_node->get_parameter("use_sim_time").as_bool();
    x = (float)(base_node->get_parameter("x").as_double());
    y = (float)(base_node->get_parameter("y").as_double());
    z = (float)(base_node->get_parameter("z").as_double());
    steps = base_node->get_parameter("steps").as_string();

    // plan and execute trajectory
    execute_trajectory_plan();
  }

  ~MoveToCoords() {}

  void execute_trajectory_plan() {
    // ~~~~~~~~~~ start trajectory planning ~~~~~~~~~~ //

    // get initial pose
    update_debug_vector("Initial Pose");

    if (steps == "once") {
      set_target_pose_to_current_pose_ur3_arm();
      cartesian_waypoints.push_back(target_pose_ur3_arm);
      target_pose_ur3_arm.position.x = x;
      target_pose_ur3_arm.position.y = y;
      target_pose_ur3_arm.position.z = z;
      cartesian_waypoints.push_back(target_pose_ur3_arm);
      plan_and_execute_ur3_arm_cartesian();
      update_debug_vector("Final Pose");
    } else if ((steps == "xyz") || (steps == "xzy") || (steps == "yxz") ||
               (steps == "yzx") || (steps == "zxy") || (steps == "zyx")) {
      for (int i = 0; i < 3; i++) {
        if (steps[i] == 'x') {
          plan_and_execute_waypoints_target(x, "x");
        } else if (steps[i] == 'y') {
          plan_and_execute_waypoints_target(y, "y");
        } else if (steps[i] == 'z') {
          plan_and_execute_waypoints_target(z, "z");
        } else {
          // just proceed - nothing to do!
        }
      }
      update_debug_vector("Final Pose");
    } else {
      RCLCPP_INFO(LOGGER, "Invalid \"steps\" Argument. Program Terminated!");
    }

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

  // declare move_group_interface varibles for ur3_arm
  std::shared_ptr<MoveGroupInterface> move_group_ur3_arm;

  // declare joint_model_group for ur3_arm
  const JointModelGroup *joint_model_group_ur3_arm;

  // declare trajectory planning variables for ur3_arm
  std::vector<double> joint_group_positions_ur3_arm;
  RobotStatePtr current_state_ur3_arm;
  Plan trajectory_plan_ur3_arm;
  Pose current_pose_ur3_arm;
  Pose target_pose_ur3_arm;
  bool plan_success_ur3_arm = false;

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
  float x, y, z;
  std::string steps;

  void get_current_state_ur3_arm() {
    // get current state of ur3_arm
    current_state_ur3_arm = move_group_ur3_arm->getCurrentState(10);
    current_state_ur3_arm->copyJointGroupPositions(
        joint_model_group_ur3_arm, joint_group_positions_ur3_arm);
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
}; // class MoveToCoords

int main(int argc, char **argv) {
  // initialize program node
  rclcpp::init(argc, argv);

  // initialize nase_node as shared pointer
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("move_to_coords");

  // instantiate class
  MoveToCoords move_to_coords_node(base_node);

  // execute trajectory plan
  move_to_coords_node.execute_trajectory_plan();

  // shutdown ros2 node
  rclcpp::shutdown();

  return 0;
}

// End of Code
