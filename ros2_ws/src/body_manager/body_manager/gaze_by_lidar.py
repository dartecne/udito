import numpy as np
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from body_interfaces.srv import ComActMsg
from body_interfaces.msg import DOA
from body_interfaces.srv import Behavior
from body_interfaces.msg import SequencerMsg

class LidarMinDistanceNode(Node):
    def __init__(self):
        super().__init__('lidar_min_distance_node')
        self.cli_seq = self.create_client(Behavior, 'sequencer_server')    
        self.cli_com_act = self.create_client(ComActMsg, 'com_act_server')
        if not self.cli_seq.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('sequencer_server not available, ignoring...')

        while not self.cli_com_act.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('multimodal_expression_server not available, waiting again...')

        self.req = Behavior.Request()
        self.req.id = "GAZE_BY_LIDAR"

        self.com_act_req = ComActMsg.Request()

        self.sequencer_subscription = self.create_subscription(
            SequencerMsg,
            'sequencer_topic',
            self.sequencer_callback,
            10) 

        self.subscription = self.create_subscription(
            LaserScan,
            '/ldlidar_node/scan',  # Ajusta el topic si es diferente
            self.lidar_callback,
            10)
        self.subscription  # Evita la advertencia de variable no usada
        self.active = True

    def sequencer_callback(self, msg):
        self.get_logger().info('Sequencer publishes\nstate: %s \nparam:%d' %(msg.state,msg.param))
        if msg.state == "GAZE_BY_LIDAR":
            self.active = True

    def print_values(self, msg):
        for i, r in enumerate(msg.ranges):
            self.get_logger().info(f'[{i}] {r:.2f} m - {np.degrees(msg.angle_min + i * msg.angle_increment):.2f}°')

    def lidar_callback(self, msg):
#        self.print_values(msg)
    
        # Define el rango de interés (en grados)
        angle_min_deg = 135  # Límite inferior en grados
        angle_max_deg = 225   # Límite superior en grados
        self.data_old = 0
        
        # Convierte ángulos a índices
        angle_min_idx = int((angle_min_deg - np.degrees(msg.angle_min)) / np.degrees(msg.angle_increment))
        angle_max_idx = int((angle_max_deg - np.degrees(msg.angle_min)) / np.degrees(msg.angle_increment))
        
        # Extrae el rango de distancias
        ranges = np.array(msg.ranges[angle_min_idx:angle_max_idx])
        
        # Filtra valores no válidos
        ranges = np.array(msg.ranges)
        ranges = ranges[~np.isnan(ranges)]
#        ranges = ranges[ranges > msg.range_min]
#        ranges = ranges[ranges < msg.range_max]
        ranges = ranges[ranges > 0.4]
        ranges = ranges[ranges < 2.5]
        
        if ranges.size > 0:
            min_distance = np.min(ranges)
            min_index = np.where(msg.ranges == min_distance)[0][0]
            min_angle = np.degrees(msg.angle_min + min_index * msg.angle_increment)

            max_distance = np.max(ranges)
            max_index = np.where(msg.ranges == max_distance)[0][0]
            max_angle = np.degrees(msg.angle_min + max_index * msg.angle_increment)

            self.get_logger().info(f'Distancia mínima: {min_distance:.2f} m a {min_angle:.2f}°')
            self.get_logger().info(f'Distancia máxima: {max_distance:.2f} m a {max_angle:.2f}°')
            text = ""
            gesture = "PAN"
            data = math.floor(min_angle - 180)
            if(abs(data - self.data_old) > 10):
                self.send_com_act(text, gesture, data) 
                time.sleep(0.5)
            self.data_old = data
        else:
            self.get_logger().info('No se encontraron valores válidos en el rango seleccionado.')

    def send_com_act(self, text, gesture, data):
        self.com_act_req.text = text
        self.com_act_req.gesture = gesture
        self.com_act_req.data = data
        self.speaking = True
        self.get_logger().info(
            'Sending to com_act_server: for %s' %self.com_act_req.text) 
        future = self.cli_com_act.call_async(self.com_act_req)
        future.add_done_callback(self.com_act_service_callback) 

    def com_act_service_callback( self, future ):
        try:
            response = future.result()
            self.get_logger().info(
                        'Result of ComAct service: for %s  = %s' %                                
                        (self.com_act_req.text, response.rta)) 
        except Exception as e:
            self.get_logger().info(
                        'Service call failed %r' % (e,))    

def main(args=None):
    rclpy.init(args=args)
    node = LidarMinDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

## Single scan from a planar laser range-finder
##
## If you have another ranging device with different behavior (e.g. a sonar
## array), please find or create a different message, since applications
## will make fairly laser-specific assumptions about this data
#
#std_msgs/Header header # timestamp in the header is the acquisition time of
#        builtin_interfaces/Time stamp
#                int32 sec
#                uint32 nanosec
#        string frame_id
#                             # the first ray in the scan.
#                             #
#                             # in frame frame_id, angles are measured around
#                             # the positive Z axis (counterclockwise, if Z is up                                    )
#                             # with zero angle being forward along the x axis
#
#float32 angle_min            # start angle of the scan [rad]
#float32 angle_max            # end angle of the scan [rad]
#float32 angle_increment      # angular distance between measurements [rad]
#
#float32 time_increment       # time between measurements [seconds] - if your sca                                    nner
#                             # is moving, this will be used in interpolating pos                                    ition
#                             # of 3d points
#float32 scan_time            # time between scans [seconds]
#
#float32 range_min            # minimum range value [m]
#float32 range_max            # maximum range value [m]
#
#float32[] ranges             # range data [m]
#                             # (Note: values < range_min or > range_max should b                                    e discarded)
#float32[] intensities        # intensity data [device-specific units].  If your
#                             # device does not provide intensities, please leave
#                             # the array empty.
