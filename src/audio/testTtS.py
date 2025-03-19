from TtS import TtS

myTtS = TtS()
myTtS.speak("Hola ¿cómo estás?")
myTtS.tts_to_file("Inicializando sistema...")

print("TTS en tiempo real. Escribe 'salir' para terminar.")
while True:
    text = input("Escribe el texto que quieres convertir en voz: ")
    if text.lower() == "salir":
        print("Saliendo del programa.")
        break
    elif text.lower() == "speaker":
        print("Speakers disponibles:")
        print(myTtS.coqui_speakers_idx)
        print(myTtS.watson_speakers_idx)
        speaker = input("Escribe el nombre del speaker: ")
        myTtS.set_tts_speaker(speaker)
    try:
        # Generar audio como numpy array
#        myTtS.speak_es(text)
        myTtS.speak(text)
    except Exception as e:
        print(f"Error: {e}")
