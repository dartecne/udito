from body_interfaces.srv import ComAct

import rclpy
from rclpy.node import Node 
import sys
sys.path.append("/home/udito/OneDrive/UDITO/udito/src/audio")
from ComAct import ComAct

class ComActService(Node):
    def __init__(self):
        super().__init__('com_act_server')
        self.srv = self.create_service(ComAct, 'com_act_server', self.com_act_callback)
        self.myComAct = ComAct()
        self.myComAct.speak("Servicio de comunicación activo!")

    def com_act_callback(self, request, response):
        self.get_logger().info('Server received\ntext: %s gesture: %s gesture_parameter: %d' % (request.text, request.gesture, request.gesture_parameter))
        self.myComAct.speak(request.text, request.gesture, request.gesture_parameter)
        response.rta = "ACK"
        return response
    
def main():
    rclpy.init()
    com_act_service = ComActService()
    rclpy.spin(com_act_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
