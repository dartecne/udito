import sounddevice as sd
import numpy as np
import sys 
sys.path.insert(0, '..')

from TtS import TtS
from StT import StT 

class DMS:
    def __init__(self):
        print("DMS::ctor")
        self.StT = StT()
        self.TtS = TtS()
        self.TtS.speak("Hola soy UDITO, en qué puedo ayudarte?")
    
    def loop(self):

        audio_buffer = b''  # Buffer para almacenar audio capturado
        speaking = False    # Indicador de actividad de voz

        while(True):
            print("Esperando actividad de voz... ")
            data = self.StT.stream.read(self.StT.chunk, exception_on_overflow=False)
            # Detectar actividad de voz
            if self.StT.is_speech(data):
                if not speaking:
                    print("Usuario comenzó a hablar...")
                    speaking = True
                audio_buffer += data
            else:
                if speaking:
                    print("Usuario terminó de hablar.")
                    speaking = False
                    # Convertir el audio capturado a formato que Whisper entienda
                    audio_np = np.frombuffer(audio_buffer, dtype=np.int16).astype(np.float32) / 32768.0
                    # Transcribir el audio
                    print("Transcribiendo...")
                    self.result = self.StT.model.transcribe(audio_np, language="es")
                    print("Transcripción: ", self.result["text"])
                    # Limpiar el buffer después de procesar
                    audio_buffer = b''
                    if self.result["text"].lower() == "salir":
                        print("Saliendo del programa.")
                        break
                    try:
                        # Generar audio como numpy array
                        self.TtS.speak(self.result["text"])
                #        myTtS.speak(text)
                    except Exception as e:
                        print(f"Error: {e}")

