import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yektapanj/WareBotSim/install/urdf_tutorial_r2d2'
