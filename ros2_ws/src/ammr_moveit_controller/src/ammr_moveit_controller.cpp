#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp> 

class ArmController : public rclcpp::Node
{
public:
  static std::shared_ptr<ArmController> create()
  {
    auto node = std::shared_ptr<ArmController>(new ArmController());
    node->init();  // gọi sau khi node đã nằm trong shared_ptr
    return node;
  }

private:
  ArmController() : Node("arm_moveit_") {
    RCLCPP_INFO(this->get_logger(), "ArmController node constructed");
  }

  void init()
  {
    // Khởi tạo MoveGroupInterface
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "arm");
    move_group_->setGoalPositionTolerance(0.01);        // THÊM
    move_group_->setGoalOrientationTolerance(0.01);     // THÊM
    move_group_->setPlanningTime(5.0);     

    // Tạo subscriber
    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/arm_control", 10,
      std::bind(&ArmController::poseCallback, this, std::placeholders::_1)
    );

    status_pub_ = this->create_publisher<std_msgs::msg::String>("/arm_status", 10);

    RCLCPP_INFO(this->get_logger(), "ArmController initialized, waiting for /arm_control topic...");
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received pose, planning...");

    move_group_->setPoseTarget(msg->pose);
    move_group_->setPlannerId("RRTConnectkConfigDefault"); 
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    std_msgs::msg::String status_msg;
    auto traj = plan.trajectory_.joint_trajectory;
    RCLCPP_INFO(this->get_logger(), "Trajectory has %zu points", traj.points.size());

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Executing...");
      status_msg.data = "success";
      // move_group_->execute(plan);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed");
      status_msg.data = "error: planning failed";
    }

    status_pub_->publish(status_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = ArmController::create();  // dùng factory pattern
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
