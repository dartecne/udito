import sys

from body_interfaces.srv import Text2Speech
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Text2Speech, 'tts')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Text2Speech.Request()

    def send_request(self, text, speaker):
        self.req.text = text
        self.req.speaker = speaker
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request("Esto es una frase de prueba", "")
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
        'Result of tts: %s' % response.rta)
    
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()