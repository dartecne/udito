# Communicative Multimodal Act
# 
import threading
from TtS import TtS

import sys
sys.path.append("/home/udito/OneDrive/UDITO/udito/src/head")
from headClass import Head

class ComAct:
    def __init__(self):
        print("ComAct::ctor")
        self.tts = TtS()
        self.head = Head()
        self.tts_thread = threading.Thread(target = self.tts_thread_function, args=(1,))
        self.gesture_thread = threading.Thread(target = self.gesture_thread_function, args=(1,))
        self.audio_data = self.tts.get_audio_data("hola")
        self.audio_data_len = len(self.audio_data)
        self.text = ""
        self.gesture = ""
        self.gesture_parameter = 0

    def speak(self, text):
        self.speak(text, "NEUTRAL", 5)

    def speak(self, text, gesture, gesture_parameter):
        if text == "":
            text = "aha"
        self.text = text
        self.gesture = gesture
        self.gesture_parameter = gesture_parameter
        self.audio_data = self.tts.get_audio_data(text)
        self.audio_data_len = len(self.audio_data)
        print(f"audio_data_len: {self.audio_data_len}")
        self.head.parse_gesture(self.gesture, self.gesture_parameter)
        self.tts.write_audio_data(self.audio_data)
#        self.tts_thread.start()
#        self.gesture_thread.start()
#        self.gesture_thread.join()

    def tts_thread_function(self, name): 
        self.tts.write_audio_data(self.audio_data)

    def gesture_thread_function(self, name):
        self.head.parse_gesture(self.gesture, self.gesture_parameter)
        self.tts_thread.join()

    def shut_up(self):
        self.audio_device.stop_stream()

    def close(self):
        self.audio_device.close()
        self.p.terminate()
