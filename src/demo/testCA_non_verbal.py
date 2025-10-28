import sys
import time
sys.path.insert(0,'../audio/')

from ComAct import ComAct
import time

myCA = ComAct()
myCA.non_verbal_expression("ANGRY", 5)
time.sleep(1)
myCA.non_verbal_expression("LOVE", 5)
time.sleep(1)
myCA.non_verbal_expression("LAUGH", 5)
time.sleep(1)
myCA.non_verbal_expression("SAD", 5)
time.sleep(1)
time.sleep(1)
myCA.non_verbal_expression("HAPPY", 5)
time.sleep(1)
myCA.non_verbal_expression("SURPRISED", 5)
time.sleep(1)
myCA.non_verbal_expression("NEUTRAL", 5)
time.sleep(1)
myCA.non_verbal_expression("BLINK", 5)
time.sleep(1)
myCA.non_verbal_expression("WINK", 5)
time.sleep(1)
myCA.non_verbal_expression("YES", 5)
time.sleep(1)
myCA.non_verbal_expression("NO", 5)
time.sleep(1)

myCA.close()