from numpy import save
import rospy
import sys, select, termios, tty
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist
import tf
import math
from threading import Thread
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
import os

orientation = None
positionX = None
positionY = None
referenceFrame = "/map"
robotBaseFrame = "/base_footprint"
stateSaveUse2DEstimate = False
stateNewPosition = False
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
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def callbackInitialPose(msg:PoseWithCovarianceStamped):
    global orientation
    global positionX
    global positionY
    global stateNewPosition

    if(stateSaveUse2DEstimate):
        positionX = msg.pose.pose.position.x
        positionY = msg.pose.pose.position.y
        q = (msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

        orientation = math.degrees(euler_from_quaternion(q)[2])
        stateNewPosition = True

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

def getRobotLocation():
    while not rospy.is_shutdown():
        try:
            (pose,q) = tfListener.lookupTransform(referenceFrame, robotBaseFrame, rospy.Time(0))
            euler = euler_from_quaternion(q)
            orientation = math.degrees(euler[2])
            positionX = pose[0]
            positionY = pose[1]
            return positionX, positionY, orientation
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Fail to get transform")

def loopUpdateRobotLocation():
    while 1:
        updateRobotLocation()
        time.sleep(0.1)

def showOption():
    print("\nOption:")
    print("0: exit")
    print("1: append current robot position into waypoint file")
    print("2: 2D Estimate Mode")
    print("3: clear waypoint data value")
    print("4: view current saved waypoint data")

def saveOption(x, y, w):
    print("%f %f %f"%(x,y,w))
    respond = input("Save current location?(y/n) ")
    quitStatus = False
    if respond == 'y' or respond == 'Y':
        respond = input("Enable rotation calibration?(y/n): ")
        if respond == 'y' or respond == 'Y':
            doCalibration = 1
        elif respond == 'n' or respond == 'N':
            doCalibration = 0
        else:
            print("Please input the correct option...")
            quitStatus = True

        if not quitStatus:
            respond = input("Sanitized?(y/n): ")
            if respond == 'Y' or respond == 'y':
                doSanitize = 1
            elif respond == 'N' or respond == 'n':
                doSanitize = 0
            else:
                print("Please input the correct option...")
                quitStatus = True

        if not quitStatus:
            log.write("%f,%f,%f,%d,%d"%(x,y,w,doCalibration,doSanitize))
            print("%f,%f,%f,%d,%d have been appended into waypoint file"%(x,y,w,doCalibration,doSanitize))

rospy.init_node("save_location")
tfListener = tf.TransformListener()
# thread_updatePosition = Thread(target=loopUpdateRobotLocation, daemon=True)
# thread_updatePosition.start()
log = logger("waypoints.txt")
rospy.Subscriber("initialpose", PoseWithCovarianceStamped, callbackInitialPose)
i = 0

showOption()

while not rospy.is_shutdown():
    key = getKey(None)

    if key == '1':
        x, y, w = getRobotLocation()
        saveOption(x,y,w)

    elif key == '2':
        print("Please use 2D Estimate to determine location ...")
        print("Press q to quit this function...")
        stateSaveUse2DEstimate = True
        while stateSaveUse2DEstimate and not rospy.is_shutdown():
            key = getKey(0.01)
            if key == 'q':
                break

            if(stateNewPosition):
                saveOption(positionX, positionY, orientation)
                stateNewPosition = False

    elif key == '3':
        log.clear()
        print("waypoint data has been cleared...")

    elif key == '4':
        log.read()
    # print(f"{key!r}")

    elif (key == '\x03' or key == '0'): 
        print("Bye .....")
        break

    showOption()