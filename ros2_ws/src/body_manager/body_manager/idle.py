import sys
import rclpy
import time
from random import randint
from rclpy.node import Node
from body_interfaces.srv import HeadMove
from body_interfaces.msg import DOA
from body_interfaces.srv import Behavior

class Idle(Node):
    def __init__(self):
        super().__init__('idle')
        self.srv = self.create_service(Behavior, 'idle_behavior', self.idle_callback)    
        self.cli = self.create_client(HeadMove, 'head_server')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('head_server not available, waiting again...')
        self.req = HeadMove.Request()
        self.active = 1

    def idle_callback(self, request, response):
        self.get_logger().info('Incoming request\nactive: %d' %request.active)  
        self.active = request.active
        if request.active == 1:
            self.get_logger().info('Idle active')
        else:
            self.get_logger().info('Idle inactive')
        response.rta = "ACK"    
        return response
    
    def move_head(self, cmd, data):
        self.req.command = cmd
        self.req.data = data
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.service_response_callback)


    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                        'Result of service: for %s  = %s' %                                # CHANGE
                        (self.req.data, response.rta))  # CHANGE
        except Exception as e:
            self.get_logger().info(
                        'Service call failed %r' % (e,))     
    def loop(self):
        if(self.active == 0):
            return
        while rclpy.ok():
            p = randint(-30,30)
            t1 = randint(-5,5)
            t2 = randint(-5,5)
            tau = randint(10,60)
            d = randint(100, 2000)
            self.move_head("R_TILT", t1)
            self.move_head("L_TILT", t2)
            self.move_head("PAN", p)
            self.move_head("BLINK", tau)
            time.sleep(d/1000)
            rclpy.spin_once(self)

               
def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def main(args=None):
    rclpy.init()
    idle_node = Idle()
    idle_node.loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



