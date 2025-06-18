# Communicative Multimodal Act
# 
import time
import threading
from TtS import TtS
from playsound import playsound


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

    def non_verbal_expression(self, gesture, gesture_parameter):
        self.gesture = gesture
        sound_file = "./samples/whistle-short-01.wav"
        if gesture == "ANGRY":
            sound_file = "./samples/angry_lion.wav"
        elif gesture == "LOVE":
            sound_file = "./samples/i_love_you.mp3"
        elif gesture == "LAUGH":
            sound_file = "./samples/laughing_girls.wav"
        elif gesture == "SAD":
            sound_file = "./samples/SadR2D2.mp3"
        elif gesture == "HAPPY":
            sound_file = "./samples/ExcitedR2D2.mp3"
        elif gesture == "SURPRISED":
            sound_file = "./samples/cat_meow.mp3"
        elif gesture == "NEUTRAL":
            sound_file = "./samples/WhistleAttention.wav"
        elif gesture == "BLINK":
            sound_file = "./samples/whistle-short-01.wav"
        elif gesture == "WINK":
            sound_file = "./samples/whistle_sexy.wav"
        elif gesture == "YES":
            sound_file = "./samples/LookR2D2.mp3"
        elif gesture == "NO":
            sound_file = "./samples/SnappyR2D2.mp3"

        self.gesture_parameter = gesture_parameter
        self.head.parse_gesture(self.gesture, self.gesture_parameter)
        playsound(sound_file)

    def show_gesture(self, gesture, gesture_parameter):
        self.gesture = gesture
        self.gesture_parameter = gesture_parameter
        self.head.parse_gesture(self.gesture, self.gesture_parameter)

    def speak(self, text):
        self.speak(text, "NEUTRAL", 5)

    def speak(self, text, gesture, gesture_parameter):
        self.text = text
        self.gesture = gesture
        self.gesture_parameter = gesture_parameter
        self.head.parse_gesture(self.gesture, self.gesture_parameter)
        if text != "":
            self.audio_data = self.tts.get_audio_data(text)
            self.audio_data_len = len(self.audio_data)
            print(f"audio_data_len: {self.audio_data_len}")
            self.tts.write_audio_data(self.audio_data)
#        time.sleep(self.audio_data_len/11000)   

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
