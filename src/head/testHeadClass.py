import time
from headClass import Head

head = Head()
v = 60
tau = 2
for i in range(10):
    head.gesture_yes(0.5)
    time.sleep(tau)