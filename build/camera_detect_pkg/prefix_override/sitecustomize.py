import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/opt/ros/ros2_ws/install/camera_detect_pkg'
