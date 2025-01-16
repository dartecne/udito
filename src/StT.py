# Dependencies:
#!pip install pyaudio
#!pip install sounddevice
#!pip install torch
#!pip install webrtcvad
#!pip install numpy

import sounddevice as sd
import numpy as np
import whisper
import queue
import pyaudio
import wave
import webrtcvad

class StT:
    # Configuración de PyAudio
    #CHUNK = 4096    #1024              # Tamaño de cada fragmento de audio
    FORMAT = pyaudio.paInt16  # Formato de audio
    CHANNELS = 1              # Audio mono
    RATE = 16000 #11025              # Frecuencia de muestreo compatible con Whisper
    DURATION = 0.03
    BYTES_PRE_SAMPLE = 1
    chunk = DURATION * RATE  
    chunk = int(chunk)
    active = True
    
    def __init__(self):      
<<<<<<< HEAD
        print("StT::ctor");  
=======
        print("ctor");  
>>>>>>> origin/main
        # Inicializar PyAudio
        self.audio = pyaudio.PyAudio()
        # Inicializar cola para audio
        self.audio_queue = queue.Queue()
        # Crear el modelo de Whisper
        self.model = whisper.load_model("base")  # Cambiar el modelo según sea necesario
        # Inicializar VAD
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(3)  # Nivel de sensibilidad: 0 (menos sensible) a 3 (más sensible)
        # Abrir el stream para capturar el audio del micrófono
        self.stream = self.audio.open(format=self.FORMAT, 
                            channels=self.CHANNELS, 
                            rate=self.RATE, 
                            input=True, 
        #                    output=False,
        #                    input_device_index=INPUT_DEVICE_ID,
        #                    output_device_index=OUTPUT_DEVICE_ID,
                            frames_per_buffer=self.chunk)

    # Función para detectar actividad de voz
    def is_speech(self, data):
    #    print("checking if is speech...")
        return self.vad.is_speech(data, self.RATE)

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

    def loop(self):
        print("Esperando actividad de voz... ")
        
        audio_buffer = b''  # Buffer para almacenar audio capturado
        speaking = False    # Indicador de actividad de voz
        print(self.active)
        while self.active:
            # Leer audio desde el micrófono
            data = self.stream.read(self.chunk, exception_on_overflow=False)
            # Detectar actividad de voz
            if self.is_speech(data):
                if not speaking:
                    print("Usuario comenzó a hablar...")
                    speaking = True
                audio_buffer += data
            else:
                if speaking:
                    print("Usuario terminó de hablar.")
                    speaking = False
                    # Convertir el audio capturado a formato que Whisper entienda
                    audio_np = np.frombuffer(audio_buffer, dtype=np.int16).astype(np.float32) / 32768.0
                    # Transcribir el audio
                    print("Transcribiendo...")
                    self.result = self.model.transcribe(audio_np, language="es")
                    print("Transcripción: ", self.result["text"])
                    # Limpiar el buffer después de procesar
                    audio_buffer = b''
    def close(self):
        self.active = False
        # Cerrar streams y PyAudio
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
