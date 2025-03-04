import time
from headClass import Head

head = Head()
v = 60
tau = 2
print("Testing HEAD class")
time.sleep(tau)

for i in range(10):
#    head.gesture_no(20, 0.5,3)
    head.gesture_yes(0.5)
    time.sleep(tau)
head.exit()
