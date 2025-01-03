# requirementes TTS (CoquiTTS)
import os
from TTS.api import TTS
import sounddevice as sd
import numpy as np


class TtS:
    def __init__(self):
#        self.model = "tts_models/es/css10/vits"
        self.model = "tts_models/multilingual/multi-dataset/xtts_v2"
        self.tts = TTS(model_name= self.model)
        self.emotion = "neutral"
        self.speakers_idx = ['Claribel Dervla', 'Daisy Studious', 'Gracie Wise', 'Tammie Ema', 'Alison Dietlinde', 'Ana Florence', 'Annmarie Nele', 'Asya Anara', 'Brenda Stern', 'Gitta Nikolina', 'Henriette Usha', 'Sofia Hellen', 'Tammy Grit', 'Tanja Adelina', 'Vjollca Johnnie', 'Andrew Chipper', 'Badr Odhiambo', 'Dionisio Schuyler', 'Royston Min', 'Viktor Eka', 'Abrahan Mack', 'Adde Michal', 'Baldur Sanjin', 'Craig Gutsy', 'Damien Black', 'Gilberto Mathias', 'Ilkin Urbano', 'Kazuhiko Atallah', 'Ludvig Milivoj', 'Suad Qasim', 'Torcull Diarmuid', 'Viktor Menelaos', 'Zacharie Aimilios', 'Nova Hogarth', 'Maja Ruoho', 'Uta Obando', 'Lidiya Szekeres', 'Chandra MacFarland', 'Szofi Granger', 'Camilla Holmström', 'Lilya Stainthorpe', 'Zofija Kendrick', 'Narelle Moon', 'Barbora MacLean', 'Alexandra Hisakawa', 'Alma María', 'Rosemary Okafor', 'Ige Behringer', 'Filip Traverse', 'Damjan Chapman', 'Wulf Carlevaro', 'Aaron Dreschner', 'Kumar Dahl', 'Eugenio Mataracı', 'Ferran Simen', 'Xavier Hayasaka', 'Luis Moray', 'Marcos Rudaski']
        self.speaker = "Andrew Chipper"

    def tts_to_file(self, text, file_path):
        self.tts.tts_to_file(text=text, file_path=file_path)
    
    def speak(self, text):
        audio_data = self.tts.tts(text, 
                                  emotion=self.emotion, 
                                  language="es", 
                                  speaker=self.speaker)
        sd.play(audio_data, samplerate=22050)
        sd.wait()
        