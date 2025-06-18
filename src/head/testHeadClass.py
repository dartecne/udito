import time
from headClass import Head

head = Head()
v = 60
tau = 3
print("Testing HEAD class")
time.sleep(tau)

head.gesture_happy(1)
time.sleep(tau)
head.gesture_neutral(0)
time.sleep(tau)
head.gesture_happy(5)
time.sleep(tau)
head.gesture_neutral(0)
time.sleep(tau)
head.gesture_happy(10)
time.sleep(tau)
head.gesture_neutral(0)
time.sleep(tau)

head.gesture_sad(7)
time.sleep(tau)
head.gesture_neutral(0)
time.sleep(tau)
head.gesture_angry(7)
time.sleep(tau)
head.gesture_neutral(0)
time.sleep(tau)

head.exit()
