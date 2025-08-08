# drive_distance_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from std_srvs.srv import Trigger
import math

class DriveDistanceNode(Node):
    def __init__(self):
        super().__init__('drive_distance_node')
        self.pose = (0.0, 0.0, 0.0)
        self.sub_pose = self.create_subscription(Pose2D, 'robot_pose', self.pose_cb, 10)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        # Simple service interface: call Trigger with "distance" param in request? Use a simple topic approach here:
        self.create_service(Trigger, 'drive_one_meter', self.handle_drive_one_meter)
        self.get_logger().info('DriveDistanceNode ready (use service "drive_one_meter").')

    def pose_cb(self, msg):
        self.pose = (msg.x, msg.y, msg.theta)

    def handle_drive_one_meter(self, request, response):
        # For quick test this triggers 1.0 m forward at 0.18 m/s
        target_distance = 1.0
        speed = 0.18
        start_x, start_y, _ = self.pose
        self.get_logger().info(f"Driving {target_distance} m forward at {speed} m/s")
        rate = self.create_rate(10)  # 10 Hz
        while rclpy.ok():
            curr_x, curr_y, _ = self.pose
            dx = curr_x - start_x
            dy = curr_y - start_y
            dist = math.hypot(dx, dy)
            if dist >= target_distance:
                break
            t = Twist()
            t.linear.x = speed
            t.linear.y = 0.0
            t.angular.z = 0.0
            self.pub_cmd.publish(t)
            rate.sleep()
        # stop
        t = Twist()
        self.pub_cmd.publish(t)
        response.success = True
        response.message = f"Reached {target_distance:.2f} m"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DriveDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()