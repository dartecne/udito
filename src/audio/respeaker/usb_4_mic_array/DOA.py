from tuning import Tuning
import usb.core
import usb.util
import time

dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)

if dev:
    Mic_tuning = Tuning(dev)
    print(Mic_tuning.direction)
    Mic_tuning.set_vad_threshold(30 )
    while True:
        try:
            if(Mic_tuning.is_voice()):
                print( Mic_tuning.direction)
            time.sleep(0.2)
        except KeyboardInterrupt:
            break
