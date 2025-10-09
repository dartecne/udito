import random
from ComAct import ComAct
import time

myCA = ComAct()

while(1):
    r = 11*random.random()
    r = int(r)
    if r == 0:
        myCA.non_verbal_expression("ANGRY", 5)
        time.sleep(1)
    elif r == 1:
        myCA.non_verbal_expression("LOVE", 5)
        time.sleep(1)
    elif r == 2:
        myCA.non_verbal_expression("LAUGH", 5)
        time.sleep(1)
    elif r == 3:
        myCA.non_verbal_expression("SAD", 5)
        time.sleep(1)
    elif r == 4:
        myCA.non_verbal_expression("HAPPY", 5)
        time.sleep(1)
    elif r == 5:
        myCA.non_verbal_expression("SURPRISED", 5)
        time.sleep(1)
    elif r == 6:
        myCA.non_verbal_expression("NEUTRAL", 5)
        time.sleep(1)
    elif r == 7:
        myCA.non_verbal_expression("BLINK", 5)
        time.sleep(1)
    elif r == 8:
        myCA.non_verbal_expression("WINK", 5)
        time.sleep(1)
    elif r == 9:
        myCA.non_verbal_expression("YES", 5)
        time.sleep(1)
    elif r == 10:
        myCA.non_verbal_expression("NO", 5)
        time.sleep(1)
    tau = 5*random.random()
    time.sleep(tau)
myCA.close()
