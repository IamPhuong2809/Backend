#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>

using namespace moveit::task_constructor;

Task createTask(const rclcpp::Node::SharedPtr& node) {
    Task task_;
    task_.stages()->setName("Cartesian Planning");

    const std::string group = "arm";
    const std::string eef = "gripper";

	// create Cartesian interpolation "planner" to be used in various stages
	auto cartesian_interpolation = std::make_shared<solvers::CartesianPath>();
	// create a joint-space interpolation "planner" to be used in various stages
	auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

    // start from a fixed robot state
	task_.loadRobotModel(node);
	auto scene = std::make_shared<planning_scene::PlanningScene>(task_.getRobotModel());

	{
		auto& state = scene->getCurrentStateNonConst();
		state.setToDefaultValues(state.getJointModelGroup(group), "home");

		auto fixed = std::make_unique<stages::FixedState>("initial state");
		fixed->setState(scene);
		task_.add(std::move(fixed));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("Move forward to scanzone", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::msg::Vector3Stamped direction;
		direction.header.frame_id = "odom";
		direction.vector.x = 0.2;
        direction.vector.z = -0.2;
		stage->setDirection(direction);
		task_.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("adjust camera angle", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::msg::TwistStamped twist;
		twist.header.frame_id = "odom";
		twist.twist.angular.x = M_PI / 2.0;
		stage->setDirection(twist);
		task_.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("Toggle to scan position", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::msg::Vector3Stamped direction;
		direction.header.frame_id = "odom";
		direction.vector.y = -0.2;
		stage->setDirection(direction);
		task_.add(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("Scan", cartesian_interpolation);
		stage->setGroup(group);
		geometry_msgs::msg::Vector3Stamped direction;
		direction.header.frame_id = "odom";
		direction.vector.y = +0.2;
		stage->setDirection(direction);
		task_.add(std::move(stage));
	}

    return task_;
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("ammr_mtc");
	std::thread spinning_thread([node] { rclcpp::spin(node); });

	auto task = createTask(node);
	try {
		if (task.plan())
			task.introspection().publishSolution(*task.solutions().front());
	} catch (const InitStageException& ex) {
		std::cerr << "planning failed with exception\n" << ex << task;
	}

	// keep alive for interactive inspection in rviz
	spinning_thread.join();
	return 0;
}