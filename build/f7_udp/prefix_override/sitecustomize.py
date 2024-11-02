import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dev/ros2_ws/src/f7_udp/install/f7_udp'
