import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/udito/OneDrive/UDITO/udito/ros2_ws/install/head_package'
