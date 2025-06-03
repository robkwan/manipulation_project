#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2/exceptions.h>
#include <tf2/time.h> // Include for tf2::Duration
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

// program variables
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_node");
static const std::string PLANNING_GROUP_ROBOT = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class PickAndPlace {
public:
  PickAndPlace(rclcpp::Node::SharedPtr base_node_) : base_node_(base_node_) {
    RCLCPP_INFO(LOGGER, "Initializing Class: Pick And Place Trajectory...");

    // configure node options
    rclcpp::NodeOptions node_options;
    // auto-declare node_options parameters from overrides
    node_options.automatically_declare_parameters_from_overrides(true);

    // initialize move_group node
    move_group_node_ =
        rclcpp::Node::make_shared("move_group_node", node_options);
    // start move_group node in a new executor thread and spin it
    executor_.add_node(move_group_node_);
    std::thread([this]() { this->executor_.spin(); }).detach();

    // initialize move_group interfaces
    move_group_robot_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_ROBOT);
    move_group_gripper_ = std::make_shared<MoveGroupInterface>(
        move_group_node_, PLANNING_GROUP_GRIPPER);

    // get initial state of robot and gripper
    joint_model_group_robot_ =
        move_group_robot_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_ROBOT);
    joint_model_group_gripper_ =
        move_group_gripper_->getCurrentState()->getJointModelGroup(
            PLANNING_GROUP_GRIPPER);

    // print out basic system information
    RCLCPP_INFO(LOGGER, "Planning Frame: %s",
                move_group_robot_->getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End Effector Link: %s",
                move_group_robot_->getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::vector<std::string> group_names =
        move_group_robot_->getJointModelGroupNames();
    // more efficient method than std::copy() method used in the docs
    for (long unsigned int i = 0; i < group_names.size(); i++) {
      RCLCPP_INFO(LOGGER, "Group %ld: %s", i, group_names[i].c_str());
    }

    // get current state of robot and gripper
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    current_state_gripper_ = move_group_gripper_->getCurrentState(10);
    current_state_gripper_->copyJointGroupPositions(
        joint_model_group_gripper_, joint_group_positions_gripper_);

    // set start state of robot and gripper to current state
    move_group_robot_->setStartStateToCurrentState();
    move_group_gripper_->setStartStateToCurrentState();
    /*
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(base_node_->get_clock());
        tf_listener_ =
       std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Define the fixed frame and the target frame
        fixed_frame_ = "world"; // Or "odom", depending on your setup
        target_frame_ = "base_link";

        // Introduce a small delay
        rclcpp::sleep_for(std::chrono::seconds(2)); // Adjust the duration as
       needed

        // Get the position once
        getPositionOnce();
    */
    // From /robot_description topic in
    // ./ros2_ws/src/universal_robot_ros2/Universal_Robots_ROS2_Gazebo_Simulation/ur_simulation_gazebo/launch/ur_sim_starbots.launch.py

    base_link_x_ = 5.3;
    base_link_y_ = -3.5;
    base_link_z_ = 0.92;

    // indicate initialization
    RCLCPP_INFO(LOGGER, "Class Initialized: Pick And Place Trajectory");
  }

  ~PickAndPlace() {
    // indicate termination
    RCLCPP_INFO(LOGGER, "Class Terminated: Pick And Place Trajectory");
  }

  void execute_trajectory_plan() {
    RCLCPP_INFO(LOGGER, "Planning and Executing Pick And Place ...");

    RCLCPP_INFO(LOGGER, "Going to Home Position...");
    // setup the joint value target
    RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory...");
    setup_joint_value_target(+0.0000, -1.5708, +1.5708, -1.5708, -1.5708,
                             +0.0000);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory...");
    execute_trajectory_kinematics();

    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    move_group_robot_->setStartStateToCurrentState();

    RCLCPP_INFO(LOGGER, "Going to Pregrasp Position...");
    // setup the goal pose target
    RCLCPP_INFO(LOGGER, "Preparing Goal Pose Trajectory...");
    setup_goal_pose_target(+0.340, -0.020, +0.30, -0.7071, +0.7071, +0.000,
                           +0.000);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Goal Pose Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Goal Pose Trajectory...");
    execute_trajectory_kinematics();

    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // open the gripper
    RCLCPP_INFO(LOGGER, "Opening Gripper...");
    // setup the gripper target by pose name
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    setup_named_pose_gripper("gripper_open");
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Opened");

    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    RCLCPP_INFO(LOGGER, "Approaching...");
    // setup the cartesian target
    RCLCPP_INFO(LOGGER, "Preparing Cartesian Trajectory...");
    setup_waypoints_target(0.0, 0.0, -0.12);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Cartesian Trajectory...");
    plan_trajectory_cartesian();
    RCLCPP_INFO(LOGGER, "Executing Cartesian Trajectory...");
    execute_trajectory_cartesian();

    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // close the gripper
    RCLCPP_INFO(LOGGER, "Closing Gripper...");
    // setup the gripper joint value
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    setup_joint_value_gripper(+0.500);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Closed");
    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    // setup the gripper joint value
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    setup_joint_value_gripper(+0.550);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Closed");
    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    // setup the gripper joint value
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    setup_joint_value_gripper(+0.575);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Closed");
    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    // setup the gripper joint value
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    setup_joint_value_gripper(+0.600);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Closed");
    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    // setup the gripper joint value
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    setup_joint_value_gripper(+0.625);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Closed");
    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    // setup the gripper joint value
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    setup_joint_value_gripper(+0.645);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Closed");
    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    RCLCPP_INFO(LOGGER, "Retreating...");
    // setup the cartesian target
    RCLCPP_INFO(LOGGER, "Preparing Cartesian Trajectory...");
    setup_waypoints_target(0.0, 0.0, 0.3);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Cartesian Trajectory...");
    plan_trajectory_cartesian();
    RCLCPP_INFO(LOGGER, "Executing Cartesian Trajectory...");
    execute_trajectory_cartesian();

    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    RCLCPP_INFO(LOGGER, "Going to Place Load Position...");
    // get current state of robot
    current_state_robot_ = move_group_robot_->getCurrentState(10);
    current_state_robot_->copyJointGroupPositions(joint_model_group_robot_,
                                                  joint_group_positions_robot_);
    // setup the joint value target
    RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory...");
    setup_joint_value_target(
        +3.1416, joint_group_positions_robot_[1],
        joint_group_positions_robot_[2], joint_group_positions_robot_[3],
        joint_group_positions_robot_[4], joint_group_positions_robot_[5]);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory...");
    execute_trajectory_kinematics();

    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // open the gripper
    RCLCPP_INFO(LOGGER, "Opening Gripper...");
    // setup the gripper target by pose name
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    setup_named_pose_gripper("gripper_open");
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Opened");

    // wait for few seconds
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    RCLCPP_INFO(LOGGER, "Going to Home Position...");
    // setup the joint value target
    RCLCPP_INFO(LOGGER, "Preparing Joint Value Trajectory...");
    setup_joint_value_target(+0.0000, -1.5708, +1.5708, -1.5708, -1.5708,
                             +0.0000);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Joint Value Trajectory...");
    plan_trajectory_kinematics();
    RCLCPP_INFO(LOGGER, "Executing Joint Value Trajectory...");
    execute_trajectory_kinematics();

    // close the gripper
    RCLCPP_INFO(LOGGER, "Closing Gripper...");
    // setup the gripper joint value
    RCLCPP_INFO(LOGGER, "Preparing Gripper Value...");
    setup_joint_value_gripper(+0.650);
    // plan and execute the trajectory
    RCLCPP_INFO(LOGGER, "Planning Gripper Action...");
    plan_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Executing Gripper Action...");
    execute_trajectory_gripper();
    RCLCPP_INFO(LOGGER, "Gripper Closed");

    RCLCPP_INFO(LOGGER, "Pick And Place Trajectory Execution Complete");
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
  rclcpp::Node::SharedPtr base_node_;

  // declare move_group node
  rclcpp::Node::SharedPtr move_group_node_;

  // declare single threaded executor for move_group node
  rclcpp::executors::SingleThreadedExecutor executor_;

  // declare move_group_interface variables for robot and gripper
  std::shared_ptr<MoveGroupInterface> move_group_robot_;
  std::shared_ptr<MoveGroupInterface> move_group_gripper_;

  // declare joint_model_group for robot and gripper
  const JointModelGroup *joint_model_group_robot_;
  const JointModelGroup *joint_model_group_gripper_;

  // declare trajectory planning variables for robot and gripper
  std::vector<double> joint_group_positions_robot_;
  RobotStatePtr current_state_robot_;
  Plan kinematics_trajectory_plan_;
  Pose target_pose_robot_;
  bool plan_success_robot_ = false;
  std::vector<double> joint_group_positions_gripper_;
  RobotStatePtr current_state_gripper_;
  Plan gripper_trajectory_plan_;
  bool plan_success_gripper_ = false;

  // declare cartesian trajectory planning variables for robot
  std::vector<Pose> cartesian_waypoints_;
  RobotTrajectory cartesian_trajectory_plan_;
  const double jump_threshold_ = 0.0;
  const double end_effector_step_ = 0.01;
  double plan_fraction_robot_ = 0.0;

  void setup_joint_value_target(float angle0, float angle1, float angle2,
                                float angle3, float angle4, float angle5) {
    // set the joint values for each joint of robot arm
    joint_group_positions_robot_[0] = angle0; // Shoulder Pan
    joint_group_positions_robot_[1] = angle1; // Shoulder Lift
    joint_group_positions_robot_[2] = angle2; // Elbow
    joint_group_positions_robot_[3] = angle3; // Wrist 1
    joint_group_positions_robot_[4] = angle4; // Wrist 2
    joint_group_positions_robot_[5] = angle5; // Wrist 3
    move_group_robot_->setJointValueTarget(joint_group_positions_robot_);
  }

  void setup_goal_pose_target(float pos_x, float pos_y, float pos_z,
                              float quat_x, float quat_y, float quat_z,
                              float quat_w) {
    // set the pose values for end effector of robot arm
    target_pose_robot_.position.x = pos_x;
    target_pose_robot_.position.y = pos_y;
    target_pose_robot_.position.z = pos_z;
    target_pose_robot_.orientation.x = quat_x;
    target_pose_robot_.orientation.y = quat_y;
    target_pose_robot_.orientation.z = quat_z;
    target_pose_robot_.orientation.w = quat_w;
    move_group_robot_->setPoseTarget(target_pose_robot_);
  }

  void plan_trajectory_kinematics() {
    // plan the trajectory to target using kinematics
    plan_success_robot_ =
        (move_group_robot_->plan(kinematics_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_kinematics() {
    // execute the planned trajectory to target using kinematics
    if (plan_success_robot_) {
      move_group_robot_->execute(kinematics_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Kinematics Trajectory Failed !");
    }
  }

  void setup_waypoints_target(float x_delta, float y_delta, float z_delta) {
    // initially set target pose to current pose of the robot
    target_pose_robot_ = move_group_robot_->getCurrentPose().pose;
    // add the current pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
    // calculate the desired pose from delta value for the axis
    RCLCPP_INFO(LOGGER, "target_pose_robot_: x: %4.2f. y: %4.2f, z: %4.2f",
                target_pose_robot_.position.x, target_pose_robot_.position.y,
                target_pose_robot_.position.z);
    target_pose_robot_.position.x += x_delta;
    target_pose_robot_.position.y += y_delta;
    target_pose_robot_.position.z += z_delta;
    // add the desired pose to the target waypoints vector
    cartesian_waypoints_.push_back(target_pose_robot_);
  }

  void plan_trajectory_cartesian() {
    // plan the trajectory to target using cartesian path
    plan_fraction_robot_ = move_group_robot_->computeCartesianPath(
        cartesian_waypoints_, end_effector_step_, jump_threshold_,
        cartesian_trajectory_plan_);
  }

  void execute_trajectory_cartesian() {
    // execute the planned trajectory to target using cartesian path
    if (plan_fraction_robot_ >= 0.0) {
      // 0.0 to 1.0 = success and -1.0 = failure
      move_group_robot_->execute(cartesian_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Robot Cartesian Trajectory Failed !");
    }
    // clear cartesian waypoints vector
    cartesian_waypoints_.clear();
  }

  void setup_joint_value_gripper(float angle) {
    // set the joint values for each joint of gripper
    // based on values provided
    joint_group_positions_gripper_[2] = angle;
    move_group_gripper_->setJointValueTarget(joint_group_positions_gripper_);
  }

  void setup_named_pose_gripper(std::string pose_name) {
    // set the joint values for each joint of gripper
    // based on predefined pose names
    move_group_gripper_->setNamedTarget(pose_name);
  }

  void plan_trajectory_gripper() {
    // plan the gripper action
    plan_success_gripper_ =
        (move_group_gripper_->plan(gripper_trajectory_plan_) ==
         moveit::core::MoveItErrorCode::SUCCESS);
  }

  void execute_trajectory_gripper() {
    // execute the planned gripper action
    if (plan_success_gripper_) {
      move_group_gripper_->execute(gripper_trajectory_plan_);
      RCLCPP_INFO(LOGGER, "Gripper Action Command Success !");
    } else {
      RCLCPP_INFO(LOGGER, "Gripper Action Command Failed !");
    }
  }

  void getPositionOnce() {
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(5.0);
    std::string error_string;
    tf2::Duration tf2_timeout(timeout.nanoseconds());

    if (tf_buffer_->canTransform(target_frame_, fixed_frame_,
                                 tf2::TimePointZero, tf2_timeout,
                                 &error_string)) {
      try {

        geometry_msgs::msg::TransformStamped transform_stamped =
            tf_buffer_->lookupTransform(fixed_frame_, target_frame_,
                                        tf2::TimePointZero);

        double x = transform_stamped.transform.translation.x;
        double y = transform_stamped.transform.translation.y;
        double z = transform_stamped.transform.translation.z;

        RCLCPP_INFO(
            LOGGER,
            "Base Link Position (in %s frame): X = %.2f, Y = %.2f, Z = %.2f",
            fixed_frame_.c_str(), x, y, z);

        // You can store these x and y values as member variables if you need to
        // access them later
        base_link_x_ = x;
        base_link_y_ = y;
        base_link_z_ = z;

      } catch (const tf2::TransformException &ex) {
        RCLCPP_INFO(LOGGER, "Could not transform %s to %s: %s",
                    target_frame_.c_str(), fixed_frame_.c_str(), ex.what());
        // Handle the error appropriately, perhaps exit or retry later if needed
      }
    }
  }
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2_ros::Buffer::SharedPtr tf_buffer_;
  // tf2_ros::TransformListener::SharedPtr tf_listener_;
  std::string fixed_frame_;
  std::string target_frame_;
  double base_link_x_;
  double base_link_y_;
  double base_link_z_;

}; // class PickAndPlaceTrajectory

int main(int argc, char **argv) {

  // initialize program node
  rclcpp::init(argc, argv);

  // initialize base_node as shared pointer
  std::shared_ptr<rclcpp::Node> base_node =
      std::make_shared<rclcpp::Node>("pick_and_place");

  // instantiate class
  PickAndPlace pick_and_place_node(base_node);

  // execute trajectory plan
  pick_and_place_node.execute_trajectory_plan();

  // shutdown ros2 node
  rclcpp::shutdown();

  return 0;
}

// End of Code
