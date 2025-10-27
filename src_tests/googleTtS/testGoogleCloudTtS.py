from google.cloud import texttospeech

# Inicializa el cliente
client = texttospeech.TextToSpeechClient()

# Configura la solicitud
synthesis_input = texttospeech.SynthesisInput(text="Hola, ¿cómo estás?")
voice = texttospeech.VoiceSelectionParams(
    language_code="es-ES",
    name="es-ES-Wavenet-A",
    ssml_gender=texttospeech.SsmlVoiceGender.FEMALE,
)
audio_config = texttospeech.AudioConfig(
    audio_encoding=texttospeech.AudioEncoding.MP3,
    pitch=2.0,  # Configura tono emocional
    speaking_rate=1.1,  # Velocidad
)

# Genera el audio
response = client.synthesize_speech(
    input=synthesis_input, voice=voice, audio_config=audio_config
)

# Guarda el archivo de audio
with open("output.mp3", "wb") as out:
    out.write(response.audio_content)
print("¡Audio generado con éxito!")
