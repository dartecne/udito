import time
from wheels import Base

base = Base()
v = 60
tau = 2
while(1):
    base.fwd(v)
    time.sleep(tau)
    base.stop()
    time.sleep(tau)
    base.bwd(v)
    time.sleep(tau)
    base.stop()
    time.sleep(tau)
    base.rotate_left(40)
    time.sleep(tau)
    base.stop()
    time.sleep(tau)
    base.rotate_right(40)
    time.sleep(tau)
    base.stop()
    time.sleep(tau)
