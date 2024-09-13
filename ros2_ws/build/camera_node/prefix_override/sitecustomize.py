import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/godcreator333/HARI_UBUNTU_HOME/Linux_BUILDS/SensoryAI-ROS2/ros2_ws/install/camera_node'
