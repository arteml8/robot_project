import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from robot_nodes.utils.mecanum_kinematics import MecanumKinematics
from robot_nodes.utils.odometry_tracker import OdometryTracker

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        self.kinematics = MecanumKinematics()
        self.tracker = OdometryTracker(self.kinematics)

        self.pose_publisher = self.create_publisher(Pose2D, 'robot_pose', 10)
        self.create_subscription(
            Pose2D,  # You may change this if using custom message for ticks
            'encoder_ticks',
            self.encoder_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.publish_pose)

    def encoder_callback(self, msg):
        ticks = [int(msg.x), int(msg.y), int(msg.theta), 0]  # Example placeholder
        self.tracker.update_ticks(ticks)

    def publish_pose(self):
        pose = Pose2D()
        pose.x, pose.y, pose.theta = self.tracker.get_pose()
        self.pose_publisher.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()