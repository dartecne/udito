# used to callibrate PID of the base

import time
from wheels import Base

v = 60
tau = 2

base = Base()
base.stop()
time.sleep(tau)

while(1):
    base.fwd(v)
    time.sleep(tau)
    base.stop()
    base.bwd(v)
    time.sleep(tau)
    base.stop()
    time.sleep(tau)
