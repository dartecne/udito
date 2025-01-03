import sounddevice as sd
from TTS.api import TTS
import numpy as np

# Inicializa Coqui TTS
#tts = TTS(model_name="tts_models/es/css10/vits")
tts = TTS(model_name="tts_models/multilingual/multi-dataset/xtts_v2")
emotion = "happy"
speaker_idx = "Camilla Holmström"

def text_to_speech_realtime():
    print("TTS en tiempo real. Escribe 'salir' para terminar.")

    while True:
        text = input("Escribe el texto que quieres convertir en voz: ")
        if text.lower() == "salir":
            print("Saliendo del programa.")
            break

        try:
            # Generar audio como numpy array
            audio_data = tts.tts(text, 
                                 emotion=emotion, 
                                 language="es",
                                 speaker=speaker_idx)
            
            # Reproducir el audio con sounddevice
#            sd.default.device = 1  # Configura el índice de tu dispositivo de salida
            sd.play(audio_data, samplerate=22050)
            sd.wait()

        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    text_to_speech_realtime()
