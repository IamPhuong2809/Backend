import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile

class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher_node')
        
        # Create a publisher for the arm controller joint trajectory
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', QoSProfile(depth=10)
        )
        
        # Create a subscriber for the joint states (position and velocity)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            QoSProfile(depth=10)
        )

        # Initialize an empty JointTrajectory message
        self.joint_trajectory = JointTrajectory()
        self.joint_trajectory.joint_names = [
            'Manipulator_J1_Joint', 'Manipulator_J2_Joint', 'Manipulator_J3_Joint', 
            'Manipulator_J4_Joint', 'Manipulator_J5', 'Manipulator_J6'
        ]

        # Timer to periodically send the joint trajectory
        self.timer = self.create_timer(0.1, self.publish_trajectory)

    def joint_state_callback(self, msg):
        # Callback to extract position and velocity from the JointState message
        # Here, we will map the positions and velocities to our trajectory
        self.joint_trajectory_point = JointTrajectoryPoint()
        self.joint_trajectory_point.positions = msg.position
        self.joint_trajectory_point.velocities = msg.velocity
        self.joint_trajectory_point.time_from_start.sec = 1  # set trajectory duration to 1 second

        # Add this point to the trajectory
        self.joint_trajectory.points = [self.joint_trajectory_point]

        # For debugging purposes
        self.get_logger().info(f"Received Joint State: {msg.position}")

    def publish_trajectory(self):
        # Publish the trajectory message to the arm controller
        self.joint_trajectory_publisher.publish(self.joint_trajectory)
        self.get_logger().info("Publishing JointTrajectory")

def main(args=None):
    rclpy.init(args=args)
    
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
