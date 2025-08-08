# mock_planner_node.py
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import asyncio

class MockPlanner(Node):
    def __init__(self):
        super().__init__('mock_planner_node')
        self.cli = self.create_client(Trigger, 'drive_one_meter')
        self.get_logger().info('MockPlanner waiting for drive_one_meter service...')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('drive_one_meter service not available at startup.')

    async def run_plan(self):
        # Hard-coded plan: 1m forward, 1m right (we assume we might strafe with a different service),
        # For now we call same drive distance service multiple times assuming robot faces correct direction.
        steps = 3
        for i in range(steps):
            req = Trigger.Request()
            fut = self.cli.call_async(req)
            self.get_logger().info(f'Calling drive_one_meter step {i+1}/{steps}')
            await asyncio.wrap_future(fut)  # run in asyncio-friendly way
            res = fut.result()
            self.get_logger().info('Drive completed: ' + str(res.message))
            await asyncio.sleep(0.5)
        self.get_logger().info('Mock plan finished.')

def main(args=None):
    rclpy.init(args=args)
    node = MockPlanner()
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(node.run_plan())
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()