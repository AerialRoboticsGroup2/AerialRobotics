import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/loppukokeisiin/drone_sim_env/ros2_ws/install/tello_parcours'
