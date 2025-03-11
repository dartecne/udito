from body_interfaces.srv import HeadMove          
import sys
import rclpy
from rclpy.node import Node


class HeadClientAsync(Node):

    def __init__(self):
        super().__init__('head_client_async')
        self.cli = self.create_client(HeadMove, 'head_move_service')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = HeadMove.Request()

    def send_request(self, cmd, data):
        self.req.command = cmd
        self.req.data = data
        self.future = self.cli.call_async(self.req)

def main():
    rclpy.init()

    head_client = HeadClientAsync()
    cmd = "GESTURE_HAPPY"
    data = 7
    head_client.send_request(cmd, data)

    while rclpy.ok():
        rclpy.spin_once(head_client)
        if head_client.future.done():
            try:
                response = head_client.future.result()
            except Exception as e:
                head_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                head_client.get_logger().info(
                    'Result of service: for %s  = %s' %                                # CHANGE
                    (head_client.req.data, response.rta))  # CHANGE
            break

    head_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()