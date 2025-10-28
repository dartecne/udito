# 
# main.py - Fichero principal de demos sencillas sin ROS
#

import sounddevice as sd
import numpy as np
import sys 
import time
import threading
from random import random
sys.path.insert(0, '..')
from audio.StT import StT 
sys.path.insert(0,'../audio/')
from ComAct import ComAct


class Demo:
    def __init__(self):
        print("Demo::ctor")
        self.myCA = ComAct()
        self.StT = StT()
        self.myCA.speak("Hola! Muy buenas!")
        self.active = True
        self.loop_thread = threading.Thread(target = self.loop, args=(1,))
        self.blink_thread = threading.Thread(target = self.blink, args=(1,))
        self.blink_thread.start()
        self.loop_thread.start()

    def blink(self, name):
        while(self.active):
            r_p = int(7*random())
            self.myCA.head.blink(r_p)
            time.sleep(r_p)

    def loop(self, name):
        while(self.active):
            r_p = int(7*random())
#            tau = 3
            tau = 30*random()
            o = (int(20*random()))%2
            if o == 0:
                print("non_verbal")
                self.non_verbal()
            else:
                print("verbal")
                self.verbal()
            time.sleep(r_p)
            self.myCA.head.gesture_neutral(r_p)
            time.sleep(tau)
        
    def close(self):
        print("Demo::dtor")
        self.active = False
        self.myCA.close()
        self.StT.close()

    def gaze_DOA(self):
        while(self.gaze_DOA_active):
            if self.StT.user_speaking:
                angle = self.StT.respeaker.direction
                print("angle:%d" % angle)
                head_angle = 0
                if (angle > 0) & (angle < 60):
                    head_angle = 40 # RIGHT
                elif (angle >=60) & (angle < 90):
                    head_angle = 0 # CENTER
                elif (angle >= 90) & (angle <=180):
                    head_angle = -40
                else: 
                    head_angle = 0
                self.myCA.head.pan( head_angle) # RIGHT
                time.sleep(0.1)

    def verbal(self):
            r_g = int(12*random())
            r_p = int(7*random())
            r_f = int(8*random())
            gestures = ["ANGRY", "LOVE", "LAUGH", "SAD", "HAPPY", "SURPRISED", "NEUTRAL","BLINK","WINK","YES","NO","NEUTRAL"]
            frases = ["Hola",
                      "Muy buenas",
                      "Soy UDITO, Robot social de la uni!",
                      "Soy una plataforma de investigación",
                      "Estoy en proceso...",
                      "Qué tal?",
                      "Me aburro",
                      "Bienvenido!"]
            self.myCA.speak(frases[r_f], gestures[r_g], r_p)
            time.sleep(1)

    def non_verbal(self):
            r_g = int(12*random())
            r_p = int(10*random())
            gestures = ["ANGRY", "LOVE", "LAUGH", "SAD", "HAPPY", "SURPRISED", "NEUTRAL","BLINK","WINK","YES","NO","NEUTRAL"]
            self.myCA.non_verbal_expression(gestures[r_g], r_p)
            time.sleep(1)

    def __del__(self):
        try:
            self.close()
        except:
            pass

def main(args=None):
    myDemo = Demo()
    q = input("Pulse Q para terminar...")
    if q == 'q':
        print("Finalizando demo")
        myDemo.active = False
    
if __name__ == '__main__':
    main()