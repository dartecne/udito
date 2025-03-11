import sys

import rclpy
from rclpy.node import Node
from body_manager.srv import HeadMove


class HeadClientAsync(Node):

    def __init__(self):
        super().__init__('head_client_async')
        self.cli = self.create_client(HeadMove, 'head_move_clnt')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = HeadMove.Request()

    def send_request(self, cmd, data):
        self.req.command = cmd
        self.req.data = data
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    head_client = HeadClientAsync()
    cmd = "HAPPY"
    data = 7
    future = head_client.send_request(cmd, data)
    rclpy.spin_until_future_complete(head_client, future)
    response = future.result()
    head_client.get_logger().info(
        'Body Manager send: for %s, %d' %
        (cmd, data))

    head_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()