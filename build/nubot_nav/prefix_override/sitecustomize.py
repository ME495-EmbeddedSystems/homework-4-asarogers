import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/asaace00/repo/me495/hw4/src/homework4/nubot_nav/install/nubot_nav'
