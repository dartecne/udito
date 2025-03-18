<<<<<<< HEAD
##
## Test - comunication with arduino and Wheels
##

import serial
import time

puerto = '/dev/ttyACM0'  # Reemplaza por el puerto que estás utilizando (ej. COM3 en Windows o /dev/ttyUSB0 en Linux)
velocidad = 115200  # Velocidad de baudios (ajusta según tu configuración de Arduino)

ser = serial.Serial(puerto, velocidad, timeout=1)
if(ser):
    print("Connected!"); 
else:
    print("Serial.error"); 
    exit()
def send_msg( word, value ):
    orden = word + "," + value + '\r'  # Formato "WORD,value"
    ser.write(orden.encode())  # Envía la orden
    print(f"Enviado: {orden.strip()}")

# Ejemplo de envío de órdenes
try:
    while True:
        word = input("Introduce la palabra (WORD): ")  # Palabra que define la orden
        value = input("Introduce el valor (VALUE): ")  # Valor que acompaña la orden
        send_msg(word, value)
        time.sleep(1)  # Espera de 1 segundo entre órdenes

except KeyboardInterrupt:
    print("\nPrograma terminado.")
finally:
    ser.close()  # Cierra el puerto serie
=======
##
## Test - comunication with arduino and Wheels
##

import serial
import time

puerto = '/dev/ttyACM0'  # Reemplaza por el puerto que estás utilizando (ej. COM3 en Windows o /dev/ttyUSB0 en Linux)
velocidad = 115200  # Velocidad de baudios (ajusta según tu configuración de Arduino)

ser = serial.Serial(puerto, velocidad, timeout=1)
if(ser):
    print("Connected!"); 
else:
    print("Serial.error"); 
    exit()
def send_msg( word, value ):
    orden = word + "," + value + '\r'  # Formato "WORD,value"
    ser.write(orden.encode())  # Envía la orden
    print(f"Enviado: {orden.strip()}")

# Ejemplo de envío de órdenes
try:
    while True:
        word = input("Introduce la palabra (WORD): ")  # Palabra que define la orden
        value = input("Introduce el valor (VALUE): ")  # Valor que acompaña la orden
        send_msg(word, value)
        time.sleep(1)  # Espera de 1 segundo entre órdenes

except KeyboardInterrupt:
    print("\nPrograma terminado.")
finally:
    ser.close()  # Cierra el puerto serie
>>>>>>> f8ce2ec1ae480949a449280c2870e9cefa27901c
