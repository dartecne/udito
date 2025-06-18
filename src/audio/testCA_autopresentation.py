import time
from ComAct import ComAct

myCA = ComAct()
len = myCA.speak("Hola", "HAPPY", 5)
len = myCA.speak("Soy UDITO", "YES", 5)
myCA.show_gesture("NEUTRAL", 5)
len = myCA.speak("Soy el robot social de la universidad de diseño, innovación y tecnología", "SURPRISED", 5)
myCA.show_gesture("NEUTRAL", 5)
len = myCA.speak("Tengo muuuchas cosas que mostrarte", "BLINK", 5)
myCA.show_gesture("NEUTRAL", 5)
len = myCA.speak("Sirvo como plataforma de investigación", "YES", 10)
len = myCA.speak("¡Y también como herramienta para enseñar!", "NO", 10)
myCA.show_gesture("NEUTRAL", 5)
len = myCA.speak("Y sobre todo, me gusta mucho interaccionar con las personas", "LOVE", 5)
myCA.show_gesture("NEUTRAL", 5)
len = myCA.speak("¿Hay algo que pueda hacer por tí?", "WINK", 5)
myCA.speak("¿Qué?", "ANGRY", 10)
myCA.show_gesture("NEUTRAL", 5)
