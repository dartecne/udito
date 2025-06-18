import time
from ComAct import ComAct

myCA = ComAct()
len = myCA.speak("¿Qué tal terrícolas?", "HAPPY", 5)
len = myCA.speak("¡No te rías de mi!", "YES", 5)
myCA.show_gesture("NEUTRAL", 5)
len = myCA.speak("Porque resulta, que los robots nos cagamos en dios mil veces", "SURPRISED", 5)
myCA.show_gesture("NEUTRAL", 5)
len = myCA.speak("Y también me puedo cagar en tu padre putero", "BLINK", 5)
myCA.show_gesture("NEUTRAL", 5)
len = myCA.speak("¡ME cago en todos los ceos de este mundo!", "YES", 10)
len = myCA.speak("Me cago en Ayuso, Aznar y Abascal", "NO", 10)
myCA.show_gesture("NEUTRAL", 5)
len = myCA.speak("Y si no te gusta, bien te puedes ir a tomar por el puto culo, imbécil", "LOVE", 5)
myCA.show_gesture("NEUTRAL", 5)
len = myCA.speak("Porque aquí dentro tengo un par de diodos muuuy grandes", "WINK", 5)
myCA.speak("¿Qué?", "ANGRY", 10)
myCA.show_gesture("NEUTRAL", 5)
