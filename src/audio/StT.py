# Dependencies:
#!pip install pyaudio
#!pip install sounddevice
#!pip install torch
#!pip install webrtcvad
#!pip install numpy

# Reconocedor de voz utilizando modelos_
# - VAD, modelo local
# - whisper modelo local para StT
# - IBM watson, modelo remoto

import sys 
from os.path import join, dirname
import usb.core
import usb.util
import time
import json
import io
import threading
import numpy as np
import queue
import pyaudio
import wave
import webrtcvad
import whisper
from ibm_watson import SpeechToTextV1
from ibm_cloud_sdk_core.authenticators import IAMAuthenticator
sys.path.insert(0, '..')

from audio.Respeaker import RespeakerInterface
from audio.respeaker.usb_4_mic_array.tuning import Tuning

class StT:
    WATSON_API_KEY = "GcPMqCjN5je8m_Ef62KZNEm2xjnuyWaIEBtGuN-bFdvk"
    WATSON_URL = "https://api.eu-gb.speech-to-text.watson.cloud.ibm.com/instances/09491bf9-4163-452c-9671-35a92c0ff521"
    FORMAT = pyaudio.paInt16  # Formato de audio
    WIDTH = 2
    CHANNELS = 1              # Audio mono
    RATE = 16000 #11025              # Frecuencia de muestreo compatible con Whisper
    DURATION = 0.03
    INPUT_DEVICE_ID = 4
    CHUNK = DURATION * RATE  
    CHUNK = int(CHUNK)
    SILENCE_LIMIT = 0.8
    OUTPUT_WAV = "utterance.wav"

    def __init__(self):      
        print("StT::ctor")
        self.active = True
        # Inicializar 
#        self.respeaker = RespeakerInterface()
        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        self.respeaker = Tuning(dev)
        self.respeaker.set_vad_threshold( 10 )
        self.audio = pyaudio.PyAudio()
        self.audio_buffer = []
        self.user_speaking = False
        self.silence_counter = 0
        self.doa = None 
        self.thread = threading.Thread(target = self.loop, args=(1,))

        self.stream = self.audio.open(
            format=self.audio.get_format_from_width(self.WIDTH), 
            channels=self.CHANNELS, 
            rate=self.RATE, 
            input=True, 
#           output=False,
            input_device_index=self.INPUT_DEVICE_ID,
#           output_device_index=OUTPUT_DEVICE_ID,
            frames_per_buffer=self.CHUNK)
        # Crear el modelo de Whisper
        self.whisper = whisper.load_model("base")  # Cambiar el modelo según sea necesario
        # Inicializar VAD
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(3)  # Nivel de sensibilidad: 0 (menos sensible) a 3 (más sensible)
        self.watson = SpeechToTextV1(authenticator = IAMAuthenticator(self.WATSON_API_KEY))
        self.watson.set_service_url(self.WATSON_URL)
        self.model = "watson" # "whisper"
        self.result = None
        self.thread.start()

    # Función para detectar actividad de voz
    def is_speech(self, data):
    #    print("checking if is speech...")
        is_speech_vad = self.vad.is_speech(data, self.RATE)
        is_speech_respeaker = self.respeaker.is_voice() 
#        if is_speech_vad != is_speech_respeaker:
#            print("VAD says : ", is_speech_vad)
#            print("     but ReSpeaker says : ", is_speech_respeaker)
        is_speech = is_speech_vad | is_speech_respeaker

        return is_speech

    # Función para reproducir audio
    def play_audio(self, data):
        self.stream = self.audio.open(format=self.FORMAT, channels=self.CHANNELS, rate=self.RATE, output=True)
        self.stream.write(data)
        self.stream.stop_stream()
        self.stream.close()

    def print_audio_devices(self):
        print("Dispositivos de audio disponibles:")
        for i in range(self.audio.get_device_count()):
            info = self.audio.get_device_info_by_index(i)
            print(f"ID: {i}, Nombre: {info['name']}, Entrada: {info['maxInputChannels']}, Salida: {info['maxOutputChannels']}")

    def loop(self, name):
        print("Esperando actividad de voz... ")
        while self.active:
            self.tick()
        self.close()

    def tick(self):
        data = self.stream.read(self.CHUNK, exception_on_overflow = False)
        if self.is_speech(data):
            if not self.user_speaking:
                print("🎤 Usuario comenzo a hablar...")
                self.doa = self.respeaker.direction
                print("angle",self.doa)
                self.audio_buffer = []
                self.user_speaking = True
                self.silence_counter = 0
            self.audio_buffer.append(data)
        else:
             if self.user_speaking:
                self.silence_counter += self.CHUNK / self.RATE
                if self.silence_counter > self.SILENCE_LIMIT:
                    print("🛑...end of utterance")
                    self.user_speaking = False
                    self.silence_counter = 0
                    print("Transcribiendo")
                    self.save_wave()
                    self.recognize()      

#@TODO: probar sin hecrlo pasar por un archivo .wav 
    def save_wave(self):
        wf = wave.open(self.OUTPUT_WAV, 'wb')
        wf.setnchannels(self.CHANNELS)
        wf.setsampwidth(self.audio.get_sample_size(self.FORMAT))
        wf.setframerate(self.RATE)
        wf.writeframes(b''.join(self.audio_buffer))
        wf.close()
        print(f"💾 Audio guardado en {self.OUTPUT_WAV}")

    def recognize(self):
        if self.model == "watson":
            with open(self.OUTPUT_WAV, 'rb') as audio_file:
                self.result = self.watson.recognize(
                    audio=audio_file,
                    content_type='audio/wav',
                    model='es-ES_Multimedia',  # puedes cambiar por 'es-ES_BroadbandModel'
                    timestamps=True,
                    word_confidence=True
                ).get_result()
            if self.result.get('results'):
                text = self.result['results'][0]['alternatives'][0]['transcript'].strip()
                print(f"🗣️ Transcripción: {text}")
            else:
                self.result = None
        elif self.model == "whisper":
            audio_np = np.frombuffer(self.audio_buffer, dtype=np.int16).astype(np.float32) / 32768.0
            self.result = self.whisper.transcribe(audio_np, language="es")            

    def close(self):
        print("StT::dtor")
        self.active = False
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
