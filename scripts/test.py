import sys, select, termios, tty
import time
import rospy

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

settings = termios.tcgetattr(sys.stdin)
rospy.init_node("test")

while not rospy.is_shutdown():
    key = getKey(None)
    print(key)

    if (key == '\x03'):
        break