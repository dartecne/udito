import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from head_package.srv import HeadMove

import sys
sys.path.append("/home/udito/OneDrive/UDITO/udito/src/head")
from headClass import Head

class HeadService(Node):

    def __init__(self):
        super().__init__('head_service')
        self.head = Head()
        self.srv = self.create_service(HeadMove, 'head_move_srv', self.head_manager_callback)

    def head_manager_callback(self, request, response):
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        if(request.commnand == "PAN"):
            self.head.pan(request.data)
        elif(request.commnand == "TILT_LEFT"):
            self.head.tilt_left(request.data)
        elif(request.commnand == "TILT_RIGHT"): 
            self.head.tilt_right(request.data)
        elif(request.commnand == "GESTURE_HAPPY"):  
            self.head.gesture_happy(request.data)
        elif(request.commnand == "GESTURE_SAD"):
            self.head.gesture_sad(request.data)
    #...
        return response


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.head_manager = Head()

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    head_service = HeadService()
    rclpy.spin(head_service)

    minimal_publisher = MinimalPublisher()

#    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
 #   minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
