#
# clase que implemena el movimiento del cuello (3 servos) y 
# y del display de los ojos.
#

from serial import Serial
import time


class Head:
    def __init__(self):
        print( "Head - ctor()" )
        port = '/dev/ttyACM1'  # Arduino MEGA. Cuello
        baudrate = 115200
        self.ser = Serial(port, baudrate, timeout=1) 
        if(self.ser):
            print("Connected!"); 
        else:
            print("Serial.error");  
        
    def pan(self, angle):
        self.send_msg( "PAN", angle )

    # rango [-10, 10]
    # 10: mira hacia abajo
    # - 10: mira hacia arriba
    def tilt_left(self, angle):
        self.send_msg( "L_TILT", angle )

    def tilt_right(self, angle):
        self.send_msg( "R_TILT", angle )

    def gesture_yes(self, data):
        self.tilt_left(10)
        self.tilt_right(10)
        time.sleep(1)
        self.tilt_left(-10)
        self.tilt_right(-10)
        time.sleep(1)
        self.tilt_left(0)
        self.tilt_right(0)

    def send_msg(self, cmd, value):
        order = cmd + "," + str(value) + '\r'  # Format "WORD,value"
        self.ser.write(order.encode())  # Send the order
        print(f"Sent: {order.strip()}")
    
    def exit(self):
        self.ser.close()
