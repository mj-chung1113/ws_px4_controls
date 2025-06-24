import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jmj/pro_asp_ws/ws_px4_controls/install/opencv_tests'
