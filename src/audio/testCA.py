from ComAct import ComAct

myCA = ComAct()
len = myCA.say("Hola ¿cómo estás?")
print(len)

print("TTS en tiempo real. Escribe 'salir' para terminar.")
while True:
    text = input("Escribe el texto que quieres convertir en voz: ")
    if text.lower() == "salir":
        print("Saliendo del programa.")
        break
    elif text.lower() == "speaker":
        print("Speakers disponibles:")
        print(myCA.tts.coqui_speakers_idx)
        print(myCA.tts.watson_speakers_idx)
        speaker = input("Escribe el nombre del speaker: ")
        myCA.tts.set_tts_speaker(speaker)
    try:
        # Generar audio como numpy array
#        myCA.speak_es(text)
        len = myCA.say(text)
        print(len)
    except Exception as e:
        print(f"Error: {e}")
