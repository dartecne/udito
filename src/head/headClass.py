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
        d = 8
        self.t = np.arange(0, d, 1)
        #a = BounceEaseInOut(start = 0, end = 10, duration = d )
        #b = BounceEaseInOut(start = 10, end = 0, duration = d )
        a = QuadEaseInOut(start = -4, end = 10, duration = d )
        b = QuadEaseInOut(start = 10, end = -4, duration = d )
        # ease para gesto "yes"
        self.pos = list(map(a,self.t)) + list(map(b,self.t))

        port = '/dev/ttyACM0'  # Arduino MEGA. Cuello
        baudrate = 115200
        self.ser = None
        try:
            self.ser = Serial(port, baudrate, timeout=1) 
            print("Connected!")
        except Exception as e:
            print("Serial.error")  
            print(e)

    def parse_gesture(self, gesture, data):
        if gesture == "PAN":
            self.pan(data)
        elif gesture == "L_TILT":
            self.tilt_left(data)
        elif gesture == "R_TILT":
            self.tilt_right(data)
        elif gesture == "LOVE":    
            self.gesture_love(data)
        elif gesture == "LAUGH":
            self.gesture_laugh(data)
        elif gesture == "SAD":
            self.gesture_sad(data)
        elif gesture == "HAPPY":    
            self.gesture_happy(data)
        elif gesture == "ANGRY":
            self.gesture_angry(data)
        elif gesture == "SURPRISED":
            self.gesture_surprised(data)
        elif gesture == "NEUTRAL":
            self.gesture_neutral(data)
        elif gesture == "BLINK":
            self.blink(data)
        elif gesture == "WINK":
            self.send_msg("WINK",abs(data))
        elif gesture == "YES":
            self.gesture_yes(data, 2)
        elif gesture == "NO":
            self.gesture_no(data, 2)
        else:
            print("Gesture not found")

    def serial_send(self, cmd, value):
        self.send_msg(cmd, value)   

    # max pan angle:60º
    # data>0 gira a izq
    def pan(self, data):
        angle = data * 60 / 10
        self.send_msg( "PAN", angle )

    # rango [-10, 10]
    # 10: mira hacia abajo
    # - 10: mira hacia arriba
    def tilt_left(self, angle):
        self.send_msg( "L_TILT", angle )

    def tilt_right(self, angle):
        self.send_msg( "R_TILT", angle )
    
    # data [-10, 10]
    def gesture_love(self, data):
        self.pan(-data)
        self.tilt_left(-10)
        self.tilt_right(-10)
        self.send_msg("LOVE",abs(data))
        time.sleep(1)
        self.tilt_left(data/2-5)
        self.tilt_right(-data/2-5)

    def gesture_laugh(self, data):
        self.pan(data)
        self.tilt_left(-10)
        self.tilt_right(-10)
        self.send_msg("LAUGH",abs(data))
        time.sleep(1)
        for p in self.pos:
            self.tilt_left(round(p))
            self.tilt_right(round(p))
            time.sleep(abs(data)/100)

    def gesture_happy(self, data):
        self.pan(-data)
        self.tilt_left(-data)
        self.tilt_right(-data)
        self.send_msg("HAPPY",abs(data))
    
    def gesture_sad(self, data):
        self.pan(data)
        self.tilt_left(data)
        self.tilt_right(data)
        self.send_msg("SAD",abs(data))

    def gesture_angry(self, data):
        self.tilt_left(data)
        self.tilt_right(data)
        self.send_msg("ANGRY",abs(data))

    def gesture_surprised(self, data):  
        self.tilt_left(-data)
        self.tilt_right(-data)
        self.send_msg("CELEBRATION",abs(data))
    
    def gesture_neutral(self, data):
        self.pan(0)
        self.tilt_left(0)
        self.tilt_right(0)
        self.send_msg("BLINK_LINE",abs(data))
    
    def blink(self, data):
        self.send_msg("BLINK_LINE",abs(data))

    def gesture_yes( self, data, n ):
        self.send_msg("BLINK_LINE", abs(10-data))
        for i in range(n):
            for p in self.pos:
                self.tilt_left(round(p))
                self.tilt_right(round(p))
                time.sleep(abs(10-data)/100)
        self.send_msg("BLINK_LINE", 0)
    
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

    #data [-10,10]
    def gesture_no( self, data, n ):
        self.tilt_left(5)
        self.tilt_right(5)
        self.send_msg("BLINK_LINE", abs(10-data))
        amp = 8
        tau = 1 - 0.04*data
        self.send_msg("HAPPY", 0)
#        amp = 2* amp / 3
        for i in range(n):
            self.pan(amp)
            time.sleep(tau)
            self.pan(-amp)
            time.sleep(tau)
        self.pan(0)
        self.send_msg("BLINK_LINE", abs(10-data))

    def send_msg(self, cmd, value):
        order = cmd + "," + str(value) + '\r'  # Format "WORD,value"
        print(f"Sent: {order.strip()}")
        if self.ser:
            self.ser.write(order.encode())  # Send the order
            rpta = self.ser.readline()
            print(f"Head says:{rpta}")
            self.ser.flush()
        else:
            print("HEAD not connected")

    def exit(self):
        print("Closing Serial...")
        self.ser.close()
