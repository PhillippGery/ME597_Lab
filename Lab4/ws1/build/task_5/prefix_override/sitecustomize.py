import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/Gery/ME597_Lab/Lab4/ws1/install/task_5'
