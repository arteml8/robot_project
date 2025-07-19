import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robot_nodes.utils.jetson_velocity_controller import JetsonVelocityController

class VelocityControllerNode(Node):
    def __init__(self):
        super().__init__('velocity_controller_node')
        # Assuming tracker and BLE client passed in or initialized here
        self.controller = JetsonVelocityController(kinematics, tracker, ble_client)

        self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_command_callback,
            10
        )

        self.create_timer(0.1, self.update_controller)

    def velocity_command_callback(self, msg):
        self.controller.set_target_velocity(msg.linear.x, msg.linear.y, msg.angular.z)

    def update_controller(self):
        self.get_logger().info("Updating velocity controller")
        rclpy.task.asyncio.create_task(self.controller.update())

def main(args=None):
    rclpy.init(args=args)
    node = VelocityControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()