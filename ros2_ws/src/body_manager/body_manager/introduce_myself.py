import time
import sys
import rclpy
from rclpy.node import Node
from body_interfaces.srv import Text2Speech
from body_interfaces.srv import HeadMove
from body_interfaces.srv import Behavior
from body_interfaces.msg import DOA

class IntroduceMyself(Node):
    def __init__(self):
        super().__init__('introduce_myself')
        self.srv = self.create_service(Behavior, 'introduce_myself', self.introduce_myself_callback)    
        self.text_to_speech_client = self.create_client(Text2Speech, 'text_to_speech_server')
        self.head_client = self.create_client(HeadMove, 'head_server')
        while not self.text_to_speech_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('text_to_speech_server not available, waiting again...')
        while not self.head_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('head_server not available, waiting again...')
        self.t2s_req = Text2Speech.Request()
        self.head_req = HeadMove.Request() 
        self.is_speaking = False
#        self.doa_subscription = self.create_subscription(
#            DOA,
#            'topic',
#            self.doa_callback,
#            10) 
#        

    def introduce_myself_callback(self, request, response):
        self.get_logger().info('Incoming request\nactive: %d' %request.active)
        if(request.active == 1):
            self.introduce_myself()
        response.rta = "ACK"
        return response
    
    def move_head(self, cmd, data):
        self.head_req.command = cmd
        self.head_req.data = data
        future = self.head_client.call_async(self.head_req)
        future.add_done_callback(self.head_service_response_callback) 

    def head_service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                        'Result of head service: for %s  = %s' %                                # CHANGE
                        (self.head_req.data, response.rta))  # CHANGE
        except Exception as e:
            self.get_logger().info(
                        'Service call failed %r' % (e,))    
    def speak(self, text):
        self.t2s_req.text = text
        future = self.text_to_speech_client.call_async(self.t2s_req)
        future.add_done_callback(self.t2s_service_response_callback) 
        self.is_speaking = True  

    def t2s_service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                        'Result of t2s service: for %s  = %s' %                                # CHANGE
                        (self.t2s_req.text, response.rta))  # CHANGE
        except Exception as e:
            self.get_logger().info(
                        'Service call failed %r' % (e,))    

    def introduce_myself( self ):
        self.move_head("GESTURE_HAPPY", 5)
        self.speak("Hola! Soy el robot de la universidad de diseño innovación y tecnología")
        time.sleep(3)
        self.move_head("GESTURE_NEUTRAL", 5)
        self.speak("Mi nombre es UDITO")
        self.move_head("BLINK", 5)
        time.sleep(1)
        self.speak("¿Te gustaría estudiar algo relacionado con el diseño?")
        self.move_head("BLINK", 5)
        time.sleep(1)
        self.speak("¡Ojo! ¡Cuidado!")
        self.move_head("BLINK", 10)
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)

    intro_myself = IntroduceMyself()

    rclpy.spin(intro_myself)

    rclpy.shutdown()

if __name__ == '__main__':
    main()