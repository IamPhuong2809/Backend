#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("AMMR Move Group Node Initialized");

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("ammr_moveit_controller", node_options);

  // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();
  // auto spinner = std::thread([&executor]() { executor.spin(); });

 //BEGIN

  static const std::string PLANNING_GROUP = "arm";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const moveit::core::JointModelGroup* joint_model_group = 
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


  // Move in Cartesian Path
  move_group.setPoseReferenceFrame("base_link");

  geometry_msgs::msg::Pose Current_State = move_group.getCurrentPose().pose;
  std::vector<geometry_msgs::msg::Pose> waypoints;

  // waypoints.push_back(Current_State);

  geometry_msgs::msg::Pose target_pose = Current_State;

  target_pose.position.x = 0.7;
  // target_pose.position.y += 0.2;
  // target_pose3.position.z -= 0.2; 
  waypoints.push_back(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.1;
  move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  // RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
  
  my_plan.trajectory_ = trajectory;
  move_group.execute(my_plan);

    // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  return 0;
}