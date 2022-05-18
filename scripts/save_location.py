from turtle import position
import rospy
import sys, select, termios, tty
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist
import tf
import math
from threading import Thread
import time
import os

orientation = None
positionX = None
positionY = None
referenceFrame = "/map"
robotBaseFrame = "/base_footprint"
currentPath = os.path.dirname(__file__)

class logger():
    def __init__(self, filename):
        self.fileName = filename
        self.fileDirPath = os.path.dirname(__file__) + "/../waypoints"
        self.filePath = f"{self.fileDirPath}/{self.fileName}"
        if(not os.path.exists(self.fileDirPath)):
            os.makedirs(self.fileDirPath)
        self.write('', end='')
        print(f"Waypoint data will be saved on this path:")
        print(self.filePath)
        print("")

    def write(self, data, mode = 'a', end='\n'):
        data = str(data)
        with open(self.filePath, mode = mode) as f:
            f.write(f"{data}{end}")

    def clear(self):
        with open(self.filePath, mode = 'w') as f:
            f.write('')
    
    def read(self):
        with open(self.filePath, 'r') as f:
            data = f.read()
            print("\nCurrent data:")
            print(data)
            print("")

settings = termios.tcgetattr(sys.stdin)
def getKey(key_timeout):
    tty.setraw(sys.stdin)
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def updateRobotLocation():
    global orientation
    global positionX 
    global positionY

    exceptionStatus = False
    try:
        (pose,q) = tfListener.lookupTransform(referenceFrame, robotBaseFrame, rospy.Time(0))
        euler = euler_from_quaternion(q)
        orientation = math.degrees(euler[2])
        positionX = pose[0]
        positionY = pose[1]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        exceptionStatus = True

    return positionX, positionY, orientation, exceptionStatus

def loopUpdateRobotLocation():
    while 1:
        updateRobotLocation()
        time.sleep(0.1)

rospy.init_node("save_location")
tfListener = tf.TransformListener()
thread_updatePosition = Thread(target=loopUpdateRobotLocation, daemon=True)
thread_updatePosition.start()
log = logger("waypoints.txt")
i = 0
print("Option:")
print("A: append current robot position into waypoint file")
print("C: clear waypoint data value")
print("V: view current saved waypoint data")
while not rospy.is_shutdown():
    key = getKey(None)

    if key == 'A' or key == 'a':
        log.write(f"{positionX},{positionY},{orientation}")
        print(f"{positionX},{positionY},{orientation} is appended into waypoint file")
    if key == 'C' or key == 'c':
        log.clear()
        print("waypoint data has been cleared...")
    if key == 'V' or key == 'v':
        log.read()
    # print(f"{key!r}")

    if (key == '\x03'): 
        break