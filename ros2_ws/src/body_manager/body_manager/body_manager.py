import sys
import rclpy
from rclpy.node import Node
from body_interfaces.srv import HeadMove
from body_interfaces.msg import DOA

class RobotBody(Node):
    def __init__(self):
        super().__init__('robot_body')
        self.cli = self.create_client(HeadMove, 'head_move_service')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = HeadMove.Request()
        self.subscription = self.create_subscription(
            DOA,                                               
            'topic',
            self.listener_callback,
            10)
        self.subscription
        self.msg = DOA()

    def listener_callback(self, msg):
        self.get_logger().info('Angle of voice: "%d", %d' % (msg.angle, msg.vad))  
        if msg.vad == 1:
#            self.cli.send_request("PAN", map_range(msg.angle, 180, 360, 60, -60))
            if (msg.angle > 180) & (msg.angle < 240):
#                self.send_request("PAN", map_range(msg.angle, 180, 360, 60, -60))
                self.send_request("PAN", -40) # RIGHT
            elif(msg.angle > 300) & (msg.angle < 360):  
                self.send_request("PAN", 40)
            elif(msg.angle >= 240) & (msg.angle <= 300):  
                self.send_request("PAN", 0)
            elif msg.angle < 90 :  
                self.send_request("PAN", 40) # LEFT
            elif(msg.angle >= 90) & (msg.angle <= 180):  
                self.send_request("PAN", -40) # LEFT
    
    def send_request(self, cmd, data):
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

class DOASubscriber(Node):

    def __init__(self):
        super().__init__('doa_subscriber')
        self.subscription = self.create_subscription(
            DOA,                                               
            'topic',
            self.listener_callback,
            10)
        self.subscription
        self.msg = DOA()

    def listener_callback(self, msg):
        self.msg = msg
        self.get_logger().info('Angle of voice: "%d", %d' % (msg.angle, msg.vad))  

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def main(args=None):
    rclpy.init()
    body_node = RobotBody()
    rclpy.spin(body_node)
    rclpy.shutdown()

def test():
    rclpy.init()

    doa_subscriber = DOASubscriber()
    head_client = HeadClientAsync()
#    cmd = "GESTURE_HAPPY"
    cmd = "NEUTRAL"
    data = 7
    head_client.send_request(cmd, data)

    while True:
        while rclpy.ok():
            rclpy.spin_once(doa_subscriber)
            rclpy.spin_once (head_client)
            if doa_subscriber.msg.vad == 1:
                head_client.send_request("PAN", map_range(doa_subscriber.msg.angle, 180, 360, 60, -60))
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