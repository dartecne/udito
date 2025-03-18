import pyaudio

# Inicializar PyAudio
p = pyaudio.PyAudio()

# Listar dispositivos para confirmar cuál es ReSpeaker
for i in range(p.get_device_count()):
    print(p.get_device_info_by_index(i))

# Seleccionar el dispositivo ReSpeaker manualmente
stream = p.open(format=pyaudio.paInt16,
                channels=1,
                rate=16000,  # Tasa de muestreo compatible
                output=True,
                output_device_index=4)  # Cambia al índice correcto

stream.write(response.content)
stream.stop_stream()
stream.close()
p.terminate()
