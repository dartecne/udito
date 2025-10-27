import sounddevice as sd
import numpy as np
import sys 
sys.path.insert(0, '..')

from audio.TtS import TtS
from audio.StT import StT 
# watson API KEY: s5nN6X_3X70mPv3ovMpOX7xgfXpfv6Gq3P0i3wb2JLbz
class DMS:
    def __init__(self):
        print("DMS::ctor")
        self.StT = StT()
        self.TtS = TtS()
        self.TtS.speak("Hola soy UDITO, en qué puedo ayudarte?")
        self.active = True

    def loop(self):
        while(self.active):
            self.StT.tick()
            if self.StT.result != None:
                text = self.StT.result['results'][0]['alternatives'][0]['transcript'].strip()
                try:
                    self.TtS.speak(text)
                except Exception as e:
                    print(f"Error: {e}")
                self.StT.result = None

def main():
    myDMS = DMS()
    myDMS.loop()

if __name__ == '__main__':
    main()
