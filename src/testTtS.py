from TtS import TtS

myTtS = TtS()
#myTtS.speak("Hola, cómo estás?")
myTtS.tts_to_file("Hola, cómo estás?")

print("TTS en tiempo real. Escribe 'salir' para terminar.")
while True:
    text = input("Escribe el texto que quieres convertir en voz: ")
    if text.lower() == "salir":
        print("Saliendo del programa.")
        break
    try:
        # Generar audio como numpy array
#        myTtS.speak_es(text)
        myTtS.speak(text)
    except Exception as e:
        print(f"Error: {e}")
