from TTS.api import TTS

# Descarga y usa el modelo en español
tts = TTS(model_name="tts_models/es/css10/vits")

# Genera el audio en español
tts.tts_to_file(
    text="Hola, estoy muy feliz de hablar contigo.",
    file_path="output.wav"
)

print("¡Archivo de audio generado con éxito!")
