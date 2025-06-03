cmake_minimum_required(VERSION 3.8)
project(ur3e_helper_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

# generate executables

add_executable(display_current_pose 
	src/display_current_pose.cpp)
target_include_directories(display_current_pose PUBLIC include)
ament_target_dependencies(display_current_pose ament_cmake rclcpp rclcpp_action moveit_core 
	moveit_ros_planning moveit_ros_planning_interface interactive_markers control_msgs)

add_executable(move_to_blocks 
	src/move_to_blocks.cpp)
target_include_directories(move_to_blocks PUBLIC include)
ament_target_dependencies(move_to_blocks ament_cmake rclcpp rclcpp_action moveit_core 
	moveit_ros_planning moveit_ros_planning_interface interactive_markers control_msgs)

add_executable(move_to_coords 
	src/move_to_coords.cpp)
target_include_directories(move_to_coords PUBLIC include)
ament_target_dependencies(move_to_coords ament_cmake rclcpp rclcpp_action moveit_core 
	moveit_ros_planning moveit_ros_planning_interface interactive_markers control_msgs)

add_executable(reset_grasp_object 
	src/reset_grasp_object.cpp)
target_include_directories(reset_grasp_object PUBLIC include)
ament_target_dependencies(reset_grasp_object ament_cmake rclcpp rclcpp_action moveit_core 
	moveit_ros_planning moveit_ros_planning_interface interactive_markers control_msgs)

add_executable(setup_grasp_object 
	src/setup_grasp_object.cpp)
target_include_directories(setup_grasp_object PUBLIC include)
ament_target_dependencies(setup_grasp_object ament_cmake rclcpp rclcpp_action moveit_core 
	moveit_ros_planning moveit_ros_planning_interface interactive_markers control_msgs)

# install executables
install(TARGETS 
	display_current_pose 
	move_to_blocks 
	move_to_coords 
	reset_grasp_object 
	setup_grasp_object 
	DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY 
	launch
	xacro
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# End of Code
