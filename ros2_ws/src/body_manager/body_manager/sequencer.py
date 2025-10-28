import sys
import rclpy
import time
from rclpy.node import Node
from body_interfaces.srv import HeadMove
from body_interfaces.msg import DOA
from body_interfaces.srv import Behavior
from body_interfaces.msg import SequencerMsg   

class Sequencer(Node):
    def __init__(self):
        super().__init__('sequencer_node')
        self.server = self.create_service(Behavior, 'sequencer_server', self.sequencer_callback)
        self.pub = self.create_publisher(SequencerMsg, 'sequencer_topic', 10)
        self.sub = self.create_subscription(
            DOA,                                               
            'topic',
            self.doa_callback,
            10)
        self.sub  # prevent unused variable warning

        self.req = Behavior.Request()
        self.states = ["IDLE", "GAZE", "INTRODUCE_MYSELF"]
        self.current_state = "IDLE"
        self.msg = SequencerMsg()
        self.msg.state = "IDLE"
        self.msg.param = 0
        self.pub.publish(self.msg)
        self.get_logger().info('publishing.new_state: "%s", param:%d' %(self.msg.state, self.msg.param))  

    def sequencer_callback(self, request, response):
        self.get_logger().info('%s says state: %d' %(request.id,request.end))
        if request.id == "INTRODUCE_MYSELF":
            self.current_state = "IDLE"
        self.msg.state = self.current_state
        self.pub.publish(self.msg)
        self.get_logger().info('publishing.new_state: "%s", param:%d' %(self.msg.state, self.msg.param))  

        response.rta = "ACK"
        return response 

    def doa_callback(self, msg):    
        if self.current_state =="IDLE":
            self.current_state = "INTRODUCE_MYSELF"
        self.msg.state = self.current_state
        self.pub.publish(self.msg)
        self.get_logger().info('publishing.new_state: "%s", param:%d' %(self.msg.state, self.msg.param))  

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
    for i in range(50):
#        rclpy.spin_once(seq_node)
        time.sleep(0.1)
    seq_node.msg.state = "INTRODUCE_MYSELF"
    seq_node.msg.param = 0
    seq_node.pub.publish(seq_node.msg)
    seq_node.get_logger().info('publishing.new_state: "%s", param:%d' %(seq_node.msg.state, seq_node.msg.param))  
    rclpy.spin(seq_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
