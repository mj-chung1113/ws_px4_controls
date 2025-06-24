import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/acdl1/mj_ws/ws_px4_controls/install/opencv_tests'
