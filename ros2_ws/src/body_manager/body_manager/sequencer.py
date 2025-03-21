import sys
import rclpy
from rclpy.node import Node
from body_interfaces.srv import HeadMove
from body_interfaces.msg import DOA
from body_interfaces.srv import Behavior

class Sequencer(Node):
    def __init__(self):
        super().__init__('sequencer')
        self.server = self.create_service(Behavior, 'sequencer', self.sequencer_callback)

        self.gaze_client = self.create_client(Behavior, 'gaze_behavior')
        while not self.gaze_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('gaze_behavior not available, waiting again...')

        self.introduce_myself_client = self.create_client(Behavior, 'introduce_myself_behavior')        
        while not self.introduce_myself_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('introduce_myself_behavior not available, waiting again...')
        self.req = Behavior.Request()

        self.states = ["IDLE", "GAZE", "INTRODUCE_MYSELF"]
        self.current_state = "IDLE"
        self.loop()
    
    def sequencer_callback(self, request, response):
        self.get_logger().info('Incoming request\nactive: %d' %request.active)
        self.current_state = self.states[request.state]
        response.rta = "ACK"
        return response 
    
    def loop(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.current_state == "IDLE":
                self.req.active = 1
                future = self.gaze_client.call_async(self.req)
                future.add_done_callback(self.service_response_callback)

            elif self.current_state == "GAZE":
                self.req.active = 1
                future = self.gaze_client.call_async(self.req)
                future.add_done_callback(self.service_response_callback) 

            elif self.current_state == "INTRODUCE_MYSELF":
                self.introduce_myself()
                self.gaze_req.active = 1
                future = self.gaze_client.call_async(self.gaze_req)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                        'Result of service: for %d  = %s' %                                # CHANGE
                        (self.req.active, response.rta))  # CHANGE
            if response.rta == "ACK":
                self.current_state = "IDLE"
        except Exception as e:
            self.get_logger().info(
                        'Service call failed %r' % (e,))

def main(args=None):
    rclpy.init()
    seq_node = Sequencer()
    seq_node.loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()