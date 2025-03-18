#
# clase que implemena el movimiento a las ruedas
#

import serial
import time

class Base():
    def __init__(self):
        print( "base::ctor" )
        puerto = '/dev/ttyACM0'  # Reemplaza por el puerto que estás utilizando (ej. COM3 en Windows o /dev/ttyUSB0 en Linux)
        velocidad = 115200  # Velocidad de baudios (ajusta según tu configuración de Arduino)

        self.ser = serial.Serial(puerto, velocidad, timeout=1)
        if(self.ser):
            print("Connected!"); 
        else:
            print("Serial.error"); 
    # V [0,100] - Velocidad
    def fwd(self, v):
        self.send_msg( "R_DIR", 0 )
        self.send_msg( "L_DIR", 1 )
        self.send_msg( "V", v )

    def bwd(self, v):
        self.send_msg( "R_DIR", 1 )
        self.send_msg( "L_DIR", 0 )
        self.send_msg( "V", v )

    def rotate_left(self, v):
        self.send_msg( "R_DIR", 0 )
        self.send_msg( "L_DIR", 0 )
        self.send_msg( "V", v )

    def rotate_right(self, v):
        self.send_msg( "R_DIR", 1 )
        self.send_msg( "L_DIR", 1 )
        self.send_msg( "V", v )

    def stop( self ):
        self.send_msg( "V", 0 )

    def brake( self ):
        self.send_msg( "V", 0 )
        self.send_msg( "BRAKE", 1 )

    def send_msg(self, cmd, value):
        orden = cmd + "," + str(value) + '\r'  # Formato "WORD,value"
        self.ser.write(orden.encode())  # Envía la orden
        print(f"Enviado: {orden.strip()}")

    def exit(self):
        self.ser.close()