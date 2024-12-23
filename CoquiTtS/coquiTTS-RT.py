import os
from TTS.api import TTS

# Deshabilita la GPU para evitar problemas
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

try:
    # Inicializa Coqui TTS
    tts = TTS(model_name="tts_models/multilingual/multi-dataset/xtts_v2", gpu=False)
    print("Modelo cargado con éxito.")

    # Generar un archivo de audio de prueba
    tts.tts_to_file(
        text="Hola, esta es una prueba de síntesis de voz.",
        language="es",
        speaker=tts.speakers[0],
        file_path="output.wav"
    )
    print("¡Audio generado con éxito!")

except Exception as e:
    print(f"Error: {e}")
