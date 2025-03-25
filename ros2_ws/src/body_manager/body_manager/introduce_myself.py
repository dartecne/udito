import time
import sys
import rclpy
from rclpy.node import Node
from body_interfaces.srv import ComActMsg
from body_interfaces.srv import Behavior
from body_interfaces.msg import DOA
from body_interfaces.msg import SequencerMsg

class IntroduceMyself(Node):
    def __init__(self):
        super().__init__('introduce_myself')
        self.cli_seq = self.create_client(Behavior, 'sequencer_server')    
        self.cli_com_act = self.create_client(ComActMsg, 'com_act_server')

        while not self.cli_seq.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('sequencer_server not available, waiting again...')

        while not self.cli_com_act.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('multimodal_expression_server not available, waiting again...')

        self.req = Behavior.Request()
        self.req.id = "INTRODUCE_MYSELF"

        self.com_act_req = ComActMsg.Request()

        self.sequencer_subscription = self.create_subscription(
            SequencerMsg,
            'sequencer_topic',
            self.sequencer_callback,
            10) 
        self.speaking = False

    def sequencer_callback(self, msg):
        self.get_logger().info('Sequencer publishes\nstate: %s \nparam:%d' %(msg.state,msg.param))
        if msg.state == "INTRODUCE_MYSELF":
            self.introduce_myself()

    def introduce_myself( self ):
        text ="Hola! Soy el robot de la universidad de diseño innovación y tecnología"
        gesture = "GESTURE_HAPPY"
        data = 5
        self.send_com_act(text, gesture, data)

        text = "Mi nombre es UDITO"
        gesture = "GESTURE_NEUTRAL"
        data = 5
        self.send_com_act(text, gesture, data)

        text = "¿Te gustaría estudiar algo relacionado con el diseño?"
        gesture = "BLINK"
        data = 5
        self.send_com_act(text, gesture, data)

        text = "¡Ojo! ¡Cuidado!"
        gesture = "BLINK"
        data = 10
        self.send_com_act(text, gesture, data)

        self.req.end = True
        self.cli_seq.call_async(self.req)
    
    def send_com_act(self, text, gesture, data):
#        while self.speaking:
#            rclpy.spin_once(self)
#            time.sleep(0.1)
        self.com_act_req.text = text
        self.com_act_req.gesture = gesture
        self.com_act_req.data = data
#        self.speaking = True
        future = self.cli_com_act.call_async(self.com_act_req)
#        future.add_done_callback(self.com_act_service_callback) 
        rclpy.spin_until_future_complete(self,future)
        response = future.result()
        self.get_logger().info(
            'Result of com_act_client:%s' %response.rta
        )
#        self.speaking = False
    def com_act_service_callback( self, future ):
        try:
            response = future.result()
            self.get_logger().info(
                        'Result of ComAct service: for %s  = %s' %                                
                        (self.com_act_req.text, response.rta)) 
#            if response.rta == "ACK":
            self.speaking = False
        except Exception as e:
            self.get_logger().info(
                        'Service call failed %r' % (e,))    

def main(args=None):
    rclpy.init(args=args)

    intro_myself = IntroduceMyself()

    rclpy.spin(intro_myself)

    rclpy.shutdown()

if __name__ == '__main__':
    main()