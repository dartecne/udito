#
# pip install ibm-watsonx-ai
#

from ibm_watsonx_ai import APIClient
from ibm_watsonx_ai import Credentials
from ibm_watsonx_ai.foundation_models import ModelInference

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from body_interfaces.msg import DOA
from body_interfaces.msg import Speech2Text
from body_interfaces.srv import Text2Speech
from body_interfaces.srv import ComActMsg

class DMS(Node):
    api_key = "TP8qCL22zUpXkw2mHMtoAwJJZqM9ZBzKSwV_kPR60E0S"  # la de tu servicio watsonx.ai
    project_id = "b65c1eaa-91ac-4520-9a79-ce25df3876a0"  # el ID del proyecto watsonx que creaste
    region = "eu-de"
    def __init__(self):
        super().__init__('dms_node')
        creds = Credentials(
            url=f"https://{self.region}.ml.cloud.ibm.com",
            api_key=self.api_key
        )
        self.client = APIClient(creds)

        self.model = ModelInference(
            model_id="meta-llama/llama-3-3-70b-instruct",
            credentials=creds,
            project_id=self.project_id
        )
        self.prompt = "Responde siempre en castellano y devuelve SOLO las dos primeras frases"

        self.stt_sub = self.create_subscription(
            Speech2Text,
            'stt_topic',
            self.stt_callback,
            10)

        self.cli = self.create_client(ComActMsg, 'com_act_server')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ComActMsg.Request()

    def send_request(self, text, gesture, gesture_parameter):
        self.req.text = text
        self.req.gesture = gesture
        self.req.data = gesture_parameter
        return self.cli.call_async(self.req)
    
    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Respuesta del servicio:%s' %response.rta)
        except Exception as e:
            self.get_logger().error(f'Error al llamar al servicio: {e}')

    def stt_callback(self,msg):
        self.get_logger().info('text: %s, conf: %f' %(msg.text, msg.confidence))
        # expresion no verbal "pensando..."
        self.send_request("pensando...", "ANGRY", 7)
        if msg.confidence > 0.5:
            self.prompt = msg.text
        else:
            self.promt = None
            return 
        response = self.generate_text(self.prompt)
        self.get_logger().info('watson says:%s' %response)
        self.send_request(response, "YES", 7)
        self.send_request("", "SURPRISED", 10)

    def generate_text(self, prompt):
        response = self.model.generate_text(
            prompt = prompt)
        return response
#        print(json.dumps(response, indent=2))



#credentials = Credentials(
#                   url = "https://eu-de.ml.cloud.ibm.com",
#                   api_key = "TP8qCL22zUpXkw2mHMtoAwJJZqM9ZBzKSwV_kPR60E0S"
#                  )
#


def main():
    print('Hi from watson_dialog_manager.')
    rclpy.init()

    minimal_subscriber = DMS()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
