import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/Gery/ME597_Lab/Lab3/ws1/install/sim_utils'
