from numpy import rate
import sys
import rospy
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
import math
import time

rospy.init_node("imu_rotation_test")
referenceFrame = "/map"
robotBaseFrame = "/base_footprint"
mode = int(sys.argv[1])
rotationSp = float(sys.argv[2])

orientation = 0
tfListener = tf.TransformListener()

last_angle = 0
error_angle = 0
kp = 0.1
ki = 0.02
kd = 0.1
I = 0
last_error = 0

def setSpeed(x, y, w):
    pass

def updateOrientation():
    global orientation
    exceptionStatus = False
    try:
        (_,q) = tfListener.lookupTransform( referenceFrame, robotBaseFrame, rospy.Time(0))
        euler = euler_from_quaternion(q)
        orientation = math.degrees(euler[2])
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        exceptionStatus = True

    return orientation, exceptionStatus

def getAngleError(sp, current):
    error = sp - current
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    
    return error

def pid(sp, current, dt, min, max, th_I_top, th_I_bottom):
    global I
    global last_error

    error = getAngleError(sp, current)

    P = kp * error
    if abs(error) < th_I_top and abs(error) > th_I_bottom:
        I += ki * error * dt
    else:
        I = 0
    d = kd * (error - last_error) 

    if I > max: I = max
    elif I < min: I = min

    PID = P + I + d

    if PID > max: PID = max
    elif PID < min: PID = min

    last_error = error

    return PID

def pidMoveRotate(sp, current, dt, min, max, th_I_top, th_I_bottom):
    global I
    global last_error

    error = sp - current

    P = kp * error
    if abs(error) < th_I_top and abs(error) > th_I_bottom:
        I += ki * error * dt
    else:
        I = 0
    d = kd * (error - last_error) 

    if I > max: I = max
    elif I < min: I = min

    PID = P + I + d

    if PID > max: PID = max
    elif PID < min: PID = min

    last_error = error

    return PID

def moveRotate(angleRef):
    currentRotation = 0
    last_w = 0
    velPub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    twistData = Twist()

    while 1:
        last_w, ex = updateOrientation()
        if not ex: break

    last_time = time.time()
    while not rospy.is_shutdown():
        w, _ = updateOrientation()

        delta = getAngleError(w, last_w)
        currentRotation += delta
        last_w = w

        current_time = time.time()
        speedW = pidMoveRotate(angleRef, currentRotation, current_time - last_time, -1, 1, 10, 0.01)
        last_time = current_time

        twistData.angular.z = speedW
        velPub.publish(twistData)
        print(f"{speedW:.2f} {w:.2f} {currentRotation:.2f}")

def setOrientation(angleRef):
    last_time = time.time()
    velPub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    twistData = Twist()
    while not rospy.is_shutdown():
        w,_ = updateOrientation()
        current_time = time.time()
        speedW = pid(angleRef, w, current_time - last_time, -1, 1, 10, 0.01)
        last_time = current_time

        twistData.angular.z = speedW
        velPub.publish(twistData)
        print(f"{speedW:.2f} {w:.2f}")

while not rospy.is_shutdown():
    if mode == 0: 
        setOrientation(rotationSp)
    elif mode == 1:
        moveRotate(rotationSp)
    else:
        print("arg0: mode, arg1: reference angle")
        print("mode option:")
        print("0: set orientation")
        print("1: rotate with specific angle")
        break