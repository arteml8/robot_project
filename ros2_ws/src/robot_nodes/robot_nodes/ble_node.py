import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray

import asyncio
import threading
from robot_ble_client import RobotBLEClient  # Assuming the class above is saved separately

class BLERobotNode(Node):
    def __init__(self):
        super().__init__('ble_robot_node')

        self.ble_client = RobotBLEClient()
        self.encoder_pub = self.create_publisher(Int32MultiArray, 'encoder_ticks', 10)
        self.cmd_sub = self.create_subscription(String, 'robot_ble_commands', self.cmd_callback, 10)

        self.get_logger().info('🔵 BLE Robot Node Initialized.')

        # Start BLE event loop in background thread
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.run_ble_loop, daemon=True).start()

    def run_ble_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.ble_main())

    async def ble_main(self):
        self.ble_client.on_encoder_update = self.publish_encoders

        connected = await self.ble_client.connect()
        if connected:
            self.get_logger().info("✅ BLE Connected.")
            await self.ble_client.poll_encoders(200)
        else:
            self.get_logger().error("❌ Failed to connect over BLE.")

    def publish_encoders(self, tick_list):
        msg = Int32MultiArray()
        msg.data = tick_list
        self.encoder_pub.publish(msg)

    def cmd_callback(self, msg):
        command = msg.data.strip()
        self.get_logger().info(f"➡️ BLE Command: {command}")
        asyncio.run_coroutine_threadsafe(self.ble_client.send(command), self.loop)


def main(args=None):
    rclpy.init(args=args)
    node = BLERobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()