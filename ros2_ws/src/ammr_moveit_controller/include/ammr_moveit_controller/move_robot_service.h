#ifndef YOUR_PACKAGE_YOUR_HEADER_H
#define YOUR_PACKAGE_YOUR_HEADER_H

#include <moveit/move_group_interface/move_group_interface.h>
#include "ammr_moveit_controller/srv/move_robot.hpp"


class MoveRobot : public rclcpp::Node {
public:
    explicit MoveRobot(const rclcpp::NodeOptions &options);

private:
    rclcpp::Service<ammr_moveit_controller::srv::MoveRobot>::SharedPtr service_example_server_;
    std::string node_namespace_;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Executor::SharedPtr executor_;
    std::thread executor_thread_;


    void execute_request(
        const std::shared_ptr<ammr_moveit_controller::srv::MoveRobot::Request> request,
        std::shared_ptr<ammr_moveit_controller::srv::MoveRobot::Response> response);

};

#endif //YOUR_PACKAGE_YOUR_HEADER_H