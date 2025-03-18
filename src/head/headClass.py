#
# clase que implemena el movimiento del cuello (3 servos) y 
# y del display de los ojos.
#

from serial import Serial
import time
from easing_functions import *
import numpy as np

class Head:
    def __init__(self):
        print( "Head - ctor()" )
        d = 6
        self.t = np.arange(0, d, 1)
        #a = BounceEaseInOut(start = 0, end = 10, duration = d )
        #b = BounceEaseInOut(start = 10, end = 0, duration = d )
        a = QuadEaseInOut(start = -4, end = 10, duration = d )
        b = QuadEaseInOut(start = 10, end = -4, duration = d )
        # ease para gesto "yes"
        self.pos = list(map(a,self.t)) + list(map(b,self.t))

        port = '/dev/ttyACM1'  # Arduino MEGA. Cuello
        baudrate = 115200
        self.ser = Serial(port, baudrate, timeout=1) 
        if(self.ser):
            print("Connected!"); 
        else:
            print("Serial.error");  
    
    def serial_send(self, cmd, value):
        self.send_msg(cmd, value)   

    def pan(self, angle):
        self.send_msg( "PAN", angle )

    # rango [-10, 10]
    # 10: mira hacia abajo
    # - 10: mira hacia arriba
    def tilt_left(self, angle):
        self.send_msg( "L_TILT", angle )

    def tilt_right(self, angle):
        self.send_msg( "R_TILT", angle )
    
    # data [-10, 10]
    def gesture_happy(self, data):
        self.pan(-20)
        self.tilt_left(-data)
        self.tilt_right(-data)
        self.send_msg("HAPPY",data)
    
    def gesture_sad(self, data):
        self.pan(20)
        self.tilt_left(data)
        self.tilt_right(data)
        self.send_msg("SAD",data)

    def gesture_angry(self, data):
        self.tilt_left(data)
        self.tilt_right(data)
        self.send_msg("ANGRY",data)

    def gesture_surprised(self, data):  
        self.tilt_left(-data)
        self.tilt_right(-data)
        self.send_msg("CELEBRATION",data)
    
    def gesture_neutral(self, data):
        self.pan(0)
        self.tilt_left(0)
        self.tilt_right(0)
        self.send_msg("BLINK_LINE",data)
    
    def blink(self, data):
        self.send_msg("BLINK_LINE",data)

    def gesture_yes( self, data, n ):
        for i in range(n):
            for p in self.pos:
                self.tilt_left(round(p))
                self.tilt_right(round(p))
                time.sleep(data)

    def gesture_yes_old( self, data ):
        for i in range(2):
            self.tilt_left(0)
            self.tilt_right(0)
            time.sleep(data)
            self.tilt_left(5)
            self.tilt_right(5)
            time.sleep(data)
            self.tilt_left(10)
            self.tilt_right(10)
            time.sleep(data)
            self.tilt_left(5)
            self.tilt_right(5)
            time.sleep(data)


    def gesture_no( self, amp, tau, n ):
        for i in range(n):
            self.pan(amp/2)
            self.pan(amp)
            time.sleep(tau)
            self.pan(-amp/2)
            self.pan(-amp)
            time.sleep(tau)

    def send_msg(self, cmd, value):
        order = cmd + "," + str(value) + '\r'  # Format "WORD,value"
        self.ser.write(order.encode())  # Send the order
        print(f"Sent: {order.strip()}")
        rpta = self.ser.readline()
        print(f"Head says:{rpta}")
        self.ser.flush()

    def exit(self):
        print("Closing Serial...")
        self.ser.close()
