import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/spherewalker/moonrockers/ros2_ws/install/rover_power_monitor'
