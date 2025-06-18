# requirementes TTS (CoquiTTS)
# también hace TtS mediante voces de IBM-Watson
import os
from TTS.api import TTS
from ibm_watson import TextToSpeechV1
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator
from pydub import AudioSegment
import requests
import io
import pyaudio
import sounddevice as sd
import numpy as np
import websocket
import json
import threading
import base64
import time

RESPEAKER_RATE = 16000
RESPEAKER_CHANNELS = 6 # change base on firmwares, 1_channel_firmware.bin as 1 or 6_channels_firmware.bin as 6
RESPEAKER_WIDTH = 2
RESPEAKER_INDEX = 4  # refer to input device id
CHUNK = 1024
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "output.wav"

# Configuración de autenticación
api_key = "wFGvz40iMm2kOmhvIAd3TpNcwUcgL8gfrK9agNb9K_TY"
url = "https://api.au-syd.text-to-speech.watson.cloud.ibm.com/instances/e82b66a7-1179-4249-8b60-4c7003432423"
print("connecting...")
authenticator = IAMAuthenticator(api_key)

class TtS:
    def __init__(self):
        print("TtS::ctor")
        self.model = "tts_models/es/css10/vits" # 500ms de latencia
#        self.model = "tts_models/multilingual/multi-dataset/xtts_v2" # 8 segundos de latencia
#        self.model = "tts_models/multilingual/multi-dataset/xtts_v1.1" #OSError: [WinError 6] Controlador no válido
                                    # UBUNTU: error raro de pytorch 
#        self.model = "tts_models/es/mai/tacotron2-DDC"
#        self.model = "tts_models/spa/fairseq/vits" # 
        self.emotion = "neutral"
        self.coqui_speakers_idx = ['Claribel Dervla', 'Daisy Studious', 'Gracie Wise', 'Tammie Ema', 'Alison Dietlinde', 'Ana Florence', 'Annmarie Nele', 'Asya Anara', 'Brenda Stern', 'Gitta Nikolina', 'Henriette Usha', 'Sofia Hellen', 'Tammy Grit', 'Tanja Adelina', 'Vjollca Johnnie', 'Andrew Chipper', 'Badr Odhiambo', 'Dionisio Schuyler', 'Royston Min', 'Viktor Eka', 'Abrahan Mack', 'Adde Michal', 'Baldur Sanjin', 'Craig Gutsy', 'Damien Black', 'Gilberto Mathias', 'Ilkin Urbano', 'Kazuhiko Atallah', 'Ludvig Milivoj', 'Suad Qasim', 'Torcull Diarmuid', 'Viktor Menelaos', 'Zacharie Aimilios', 'Nova Hogarth', 'Maja Ruoho', 'Uta Obando', 'Lidiya Szekeres', 'Chandra MacFarland', 'Szofi Granger', 'Camilla Holmström', 'Lilya Stainthorpe', 'Zofija Kendrick', 'Narelle Moon', 'Barbora MacLean', 'Alexandra Hisakawa', 'Alma María', 'Rosemary Okafor', 'Ige Behringer', 'Filip Traverse', 'Damjan Chapman', 'Wulf Carlevaro', 'Aaron Dreschner', 'Kumar Dahl', 'Eugenio Mataracı', 'Ferran Simen', 'Xavier Hayasaka', 'Luis Moray', 'Marcos Rudaski']
        self.coqui_speaker = "Andrew Chipper"
        self.watson_speakers_idx =['es-ES_LauraV3Voice', 
                                    'es-ES_EnriqueVoice', 
                                    'es-ES_EnriqueV3Voice', 
                                    'es-LA_SofiaV3Voice', 
                                    'es-US_SofiaV3Voice']
        self.watson_speaker = "es-ES_LauraV3Voice"
        self.tts_model = TTS(model_name=self.model)

        self.tts_watson = TextToSpeechV1(authenticator=authenticator)
        self.tts_watson.set_service_url(url) 

        self.p = pyaudio.PyAudio()
        self.audio_device = self.p.open(format=pyaudio.paInt16,
                channels=1,
                rate=RESPEAKER_RATE,
                output=True)
#                output_device_index=RESPEAKER_INDEX)
        self.audio_device_type = "pyaudio" # "sounddevice"
        self.tts_engine = "watson" # "coquitts"
   
    def set_tts_audio_device(self, engine):
        self.audio_device_type = engine

    def set_tts_coqui_speaker(self, id):
        self.speaker = self.coqui_speakers_idx[id]
        self.tts_engine = "coquitts"

    def set_tts_watson_speaker(self, id):
        self.speaker = self.watson_speakers_idx[id]
        self.tts_engine = "watson"

    def set_tts_speaker(self, speaker):
        self.speaker = speaker
        if speaker in self.coqui_speakers_idx:
            self.tts_engine = "coquitts"
        elif speaker in self.watson_speakers_idx:
            self.tts_engine = "watson"

    def tts_to_file_coqui(self, text):
        self.tts_model.tts_to_file(text=text, file_path=WAVE_OUTPUT_FILENAME)

    def tts_to_file_watson(self, text):
        response = self.tts_watson.synthesize(
            text,
            voice=self.watson_speaker,
            accept='audio/wav;rate=16000'#'audio/wav'
        ).get_result()
        audio_data = response.content
        with open(WAVE_OUTPUT_FILENAME, 'wb') as audio_file:
            audio_file.write(audio_data)

    def speak(self, text):
        if text == "":
            print("No text to speak")
            return
        ad = self.get_audio_data(text)
        print(len(ad))
        self.write_audio_data(ad)
#        time.sleep(len(ad)/RESPEAKER_RATE)   
        
    def get_audio_data(self,text):
        audio_data = None
        if(self.tts_engine == "watson"):
            response = self.tts_watson.synthesize(
                text,
                voice=self.watson_speaker,
                accept='audio/l16;rate=16000'#'audio/wav'
            ).get_result()
            audio_data = response.content
        elif(self.tts_engine == "coquitts"):
            audio_data = self.tts_model.tts(text, 
                              emotion=self.emotion, 
                              language="es", 
                              speaker=self.speaker)
        return audio_data

    def write_audio_data(self, audio_data):
        self.audio_device.write(audio_data)

    def shut_up(self):
        if(self.audio_device_type == "pyaudio"):
            self.audio_device.stop_stream()

    def close(self):
        if(self.audio_device_type == "pyaudio"):
            self.audio_device.close()
            self.p.terminate()