<<<<<<< HEAD
#from tutorial_interfaces.srv import Command                                                           # CHANGE
from body_interfaces.srv import HeadMove                                                           # CHANGE
import sys
import rclpy
from rclpy.node import Node

import sys
sys.path.append("/home/udito/OneDrive/UDITO/udito/src/head")
from headClass import Head

class HeadService(Node):

    def __init__(self):
        super().__init__('head_service')
        self.srv = self.create_service(HeadMove, 'head_move_service', self.head_manager_callback)       # CHANGE
        self.head = Head()
        
    def head_manager_callback(self, request, response):
        self.get_logger().info('Incoming request\na: %s b: %d' % (request.command, request.data))
        if(request.command == "PAN"):
            self.head.pan(request.data)
        elif(request.command == "TILT_LEFT"):
            self.head.tilt_left(request.data)
        elif(request.command == "TILT_RIGHT"): 
            self.head.tilt_right(request.data)
        elif(request.command == "GESTURE_HAPPY"):  
            self.head.gesture_happy(request.data)
        elif(request.command == "GESTURE_SAD"):
            self.head.gesture_sad(request.data)
    #...
        response.rta = "ACK"
        return response

def main(args=None):
    rclpy.init(args=args)

    head_service = HeadService()

    rclpy.spin(head_service)

    rclpy.shutdown()

if __name__ == '__main__':
=======
#from tutorial_interfaces.srv import Command                                                           # CHANGE
from body_interfaces.srv import HeadMove                                                           # CHANGE
import sys
import rclpy
from rclpy.node import Node

import sys
sys.path.append("/home/udito/OneDrive/UDITO/udito/src/head")
from headClass import Head

class HeadService(Node):

    def __init__(self):
        super().__init__('head_service')
        self.srv = self.create_service(HeadMove, 'head_move_service', self.head_manager_callback)       # CHANGE
        self.head = Head()
        
    def head_manager_callback(self, request, response):
        self.get_logger().info('Incoming request\na: %s b: %d' % (request.command, request.data))
        if(request.command == "PAN"):
            self.head.pan(request.data)
        elif(request.command == "TILT_LEFT"):
            self.head.tilt_left(request.data)
        elif(request.command == "TILT_RIGHT"): 
            self.head.tilt_right(request.data)
        elif(request.command == "GESTURE_HAPPY"):  
            self.head.gesture_happy(request.data)
        elif(request.command == "GESTURE_SAD"):
            self.head.gesture_sad(request.data)
    #...
        response.rta = "ACK"
        return response

def main(args=None):
    rclpy.init(args=args)

    head_service = HeadService()

    rclpy.spin(head_service)

    rclpy.shutdown()

if __name__ == '__main__':
>>>>>>> d75813b06134359daeaea89edeabe871241fbad3
    main()