#include <memory>
#include <thread>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <moveit/robot_model/joint_model_group.h>
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "ammr_moveit_controller/move_robot_service.h"
#include <tf2_eigen/tf2_eigen.hpp>


static const double PLANNING_TIME_S = 5.0;
static const double MAX_VELOCITY_SCALE = 0.1;
static const double MAX_ACCELERATION_SCALE = 0.05;
static const unsigned int PLANNING_ATTEMPTS = 5;
static const double GOAL_TOLERANCE = 1e-3;

MoveRobot::MoveRobot(const rclcpp::NodeOptions &options)
        : Node("move_robot_service_node", options), 
          node_(std::make_shared<rclcpp::Node>("move_group_node")),
          executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()) {
    node_namespace_ = ((std::string) this->get_namespace()).erase(0, 1);
    
    service_example_server_ = this->create_service<ammr_moveit_controller::srv::MoveRobot>(
            "~/move_robot",
            std::bind(&MoveRobot::execute_request, 
            this,
            std::placeholders::_1, 
            std::placeholders::_2)
    );

    // auto mgi_options = moveit::planning_interface::MoveGroupInterface::Options(
    //         node_namespace_ + "ammr",
    //         "/" + node_namespace_,
    //         "robot_description");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "arm");
    //move_group_->setPoseReferenceFrame("base_link_inertia");
    move_group_->setPlanningTime(PLANNING_TIME_S);
    move_group_->setNumPlanningAttempts(PLANNING_ATTEMPTS);
    move_group_->setGoalTolerance(GOAL_TOLERANCE);
    move_group_->setMaxVelocityScalingFactor(MAX_VELOCITY_SCALE);
    move_group_->setMaxAccelerationScalingFactor(MAX_ACCELERATION_SCALE);
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });
}

void MoveRobot::execute_request(
    const std::shared_ptr<ammr_moveit_controller::srv::MoveRobot::Request> request,
    std::shared_ptr<ammr_moveit_controller::srv::MoveRobot::Response> response)
  {
    // Check if MoveGroupInterface is properly initialized
    if (!move_group_) {
      response->success = false;
      response->message = "MoveGroupInterface not initialized.";
      RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface is null!");
      return;
    }

    const std::string& mode = request->mode;
    const auto& target = request->target;

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    try {
      if (mode == "joint") {
        auto joint_names = move_group_->getActiveJoints();
        if (target.size() != joint_names.size()) {
          response->success = false;
          response->message = "Invalid number of joint values. Expected: " + 
                             std::to_string(joint_names.size()) + ", Got: " + 
                             std::to_string(target.size());
          return;
        }
        
        move_group_->setJointValueTarget(target);
        auto result = move_group_->plan(plan);
        bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
          auto execute_result = move_group_->execute(plan);
          success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
          response->message = success ? "Joint move executed successfully." : "Execution failed.";
        } else {
          response->message = "Planning failed for joint move.";
        }
        response->success = success;
      }

      else if (mode == "p2p") {
        if (target.size() != 7) {
          response->success = false;
          response->message = "Pose must contain 7 elements [x,y,z,qx,qy,qz,qw].";
          return;
        }

        for(uint8_t i = 0; i < 7; i++)
        {
          RCLCPP_INFO(this->get_logger(), "Target[%u]: %f", i, target[i]);
        }
        
        geometry_msgs::msg::Pose pose;
        pose.position.x = target[0];
        pose.position.y = target[1];
        pose.position.z = target[2];
        pose.orientation.x = target[3];
        pose.orientation.y = target[4];
        pose.orientation.z = target[5];
        pose.orientation.w = target[6];

        move_group_->setPoseTarget(pose);
        auto result = move_group_->plan(plan);
        bool success = (result == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
          auto execute_result = move_group_->execute(plan);
          success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
          response->message = success ? "P2P pose executed successfully." : "Execution failed.";
        } else {
          response->message = "Planning failed for P2P move.";
        }
        
        move_group_->clearPoseTargets();
        response->success = success;
      }

      else if (mode == "cartesian") {
        if (target.size() != 3) {
          response->success = false;
          response->message = "Cartesian target must be 3 elements [dx, dy, dz].";
          return;
        }
        
        move_group_->setPoseReferenceFrame("odom");
        move_group_->setEndEffectorLink("Gripper");

        std::vector<geometry_msgs::msg::Pose> waypoints;
        geometry_msgs::msg::Pose current_pose;
        
        try {
          current_pose = move_group_->getCurrentPose().pose;
        } catch (const std::exception& e) {
          response->success = false;
          response->message = "Failed to get current pose: " + std::string(e.what());
          return;
        }
        
        std::ostringstream oss; 
        oss << "Current Pose: "
            << "Position(x=" << current_pose.position.x
            << ", y=" << current_pose.position.y
            << ", z=" << current_pose.position.z
            << ") Orientation(x=" << current_pose.orientation.x
            << ", y=" << current_pose.orientation.y
            << ", z=" << current_pose.orientation.z
            << ", w=" << current_pose.orientation.w << ")";
            
        geometry_msgs::msg::Pose target_pose = current_pose;
        target_pose.position.x += target[0];
        target_pose.position.y += target[1];
        target_pose.position.z += target[2];
        waypoints.push_back(target_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.05;
        const double jump_threshold = 0.0;

        double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
        if (fraction > 0.05) {
          plan.trajectory_ = trajectory;
          auto execute_result = move_group_->execute(plan);
          bool success = (execute_result == moveit::core::MoveItErrorCode::SUCCESS);
          response->success = success;
          response->message = success ? "Cartesian path executed successfully." : "Cartesian execution failed.";
        } else {
          response->success = false;
          response->message = "Cartesian path planning failed. Fraction: " + std::to_string(fraction);
        }
      }

      else {
        response->success = false;
        response->message = "Unknown mode: " + mode + ". Valid modes: joint, p2p, cartesian";
      }

    } catch (const std::exception &e) {
      response->success = false;
      response->message = "Exception in execute_request: " + std::string(e.what());
      RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
    }
  };

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto move_group_node = std::make_shared<MoveRobot>(node_options);
    rclcpp::spin(move_group_node);

    rclcpp::shutdown();
    return 0;
}