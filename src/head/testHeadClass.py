import time
from headClass import Head

head = Head()
v = 60
tau = 4
print("Testing HEAD class")
time.sleep(2*tau)

#for i in range(10):
#    head.gesture_no(20, 0.5,3)
#    head.gesture_yes(0.5)
#    time.sleep(tau)

head.gesture_neutral(0)
time.sleep(tau)
head.gesture_happy(7)
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
