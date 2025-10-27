import sounddevice as sd
from TTS.api import TTS
import numpy as np

# Inicializa Coqui TTS
#tts = TTS(model_name="tts_models/es/css10/vits")
tts = TTS(model_name="tts_models/multilingual/multi-dataset/xtts_v2")
emotion = "sad"
speakers_idx = ['Claribel Dervla', 'Daisy Studious', 'Gracie Wise', 'Tammie Ema', 'Alison Dietlinde', 'Ana Florence', 'Annmarie Nele', 'Asya Anara', 'Brenda Stern', 'Gitta Nikolina', 'Henriette Usha', 'Sofia Hellen', 'Tammy Grit', 'Tanja Adelina', 'Vjollca Johnnie', 'Andrew Chipper', 'Badr Odhiambo', 'Dionisio Schuyler', 'Royston Min', 'Viktor Eka', 'Abrahan Mack', 'Adde Michal', 'Baldur Sanjin', 'Craig Gutsy', 'Damien Black', 'Gilberto Mathias', 'Ilkin Urbano', 'Kazuhiko Atallah', 'Ludvig Milivoj', 'Suad Qasim', 'Torcull Diarmuid', 'Viktor Menelaos', 'Zacharie Aimilios', 'Nova Hogarth', 'Maja Ruoho', 'Uta Obando', 'Lidiya Szekeres', 'Chandra MacFarland', 'Szofi Granger', 'Camilla Holmström', 'Lilya Stainthorpe', 'Zofija Kendrick', 'Narelle Moon', 'Barbora MacLean', 'Alexandra Hisakawa', 'Alma María', 'Rosemary Okafor', 'Ige Behringer', 'Filip Traverse', 'Damjan Chapman', 'Wulf Carlevaro', 'Aaron Dreschner', 'Kumar Dahl', 'Eugenio Mataracı', 'Ferran Simen', 'Xavier Hayasaka', 'Luis Moray', 'Marcos Rudaski']

def text_to_speech_realtime():
    print("TTS en tiempo real. Escribe 'salir' para terminar.")

    for spk in speakers_idx:
#        text = input("Escribe el texto que quieres convertir en voz: ")
        text = "Hola, estoy muy feliz de hablar contigo."
        if text.lower() == "salir":
            print("Saliendo del programa.")
            break
        try:
            # Generar audio como numpy array
            audio_data = tts.tts(text, emotion="sad", language="es", speaker=spk)
            print("Speaker: ", spk)
            # Reproducir el audio con sounddevice
#            sd.default.device = 1  # Configura el índice de tu dispositivo de salida
            sd.play(audio_data, samplerate=22050)
            sd.wait()

        except Exception as e:
            print(f"Error: {e}")
def text_to_speech_files():
    for spk in speakers_idx:
#        text = input("Escribe el texto que quieres convertir en voz: ")
        text = "Hola, estoy muy feliz de hablar contigo."
        try:
            # Generar audio como numpy array
            tts.tts_to_file(text, language="es", speaker=spk, file_path= "test_wav/" + spk + ".wav")

        except Exception as e:
            print(f"Error: {e}")
if __name__ == "__main__":
#    text_to_speech_realtime()
    text_to_speech_files()
    
# Seleccion:
# - Alexandra Hisakawa, dulce
# - Alison Dietlinde, infantil
# - Alma María, dulce
# - Ana Florence, madura simpatica
# - Andrew Chipper, niño majo, habla pausada
# - Annmarie Nele, sensual
# - Asya Anara, sensual
# - Badr Odhiambo, chico majo
# - Baldur Sanjin, hombre susurrante
#...
# - Eugenio Mataracı, hombre majo
# - Tammy Grit, mujer natural
# - Xabier Hayasaka, hombbre habla pausada

# - Alma María
# - Andrew Chipper, niño majo, habla pausada
# - Camila Holmstrom, mujer habla pausada
# - Eugenio Mataracı, hombre majo
# - Szofi Granger, mujer habla pausada








