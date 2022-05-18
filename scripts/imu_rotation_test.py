from numpy import rate
import sys
import rospy
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
import math
import time

rospy.init_node("imu_rotation_test")
referenceFrame = "/odom"
robotBaseFrame = "/base_footprint"
# mode = int(sys.argv[1])
# rotationSp = float(sys.argv[2])

orientation = 0
tfListener = tf.TransformListener()

last_angle = 0
error_angle = 0
kp = 0.05
ki = 0.005
kd = 0.01
I = 0
last_error = 0

pidTime = 0

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

    I_limiter_gain = 0.8
    error = getAngleError(sp, current)

    P = kp * error
    if abs(error) < th_I_top and abs(error) > th_I_bottom:
        I += ki * error * dt
    else:
        I = 0
    d = kd * (error - last_error) 

    if I > I_limiter_gain * max: I = I_limiter_gain * max
    elif I < I_limiter_gain * min: I = I_limiter_gain * min

    PID = P + I + d

    if PID > max: PID = max
    elif PID < min: PID = min

    last_error = error

    return PID

def pidMoveRotate(sp, current, dt, min, max, th_I_top, th_I_bottom):
    global I
    global last_error

    I_limiter_gain = 0.8

    error = sp - current

    P = kp * error
    if abs(error) < th_I_top and abs(error) > th_I_bottom:
        I += ki * error * dt
    else:
        I = 0
    d = kd * (error - last_error) 

    if I > I_limiter_gain * max: I = I_limiter_gain * max
    elif I < I_limiter_gain * min: I = I_limiter_gain * min

    PID = P + I + d

    if PID > max: PID = max
    elif PID < min: PID = min

    last_error = error

    return PID

def moveRotate(angleRef, maxSpeed = 1, timeOut = 5):
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
        speedW = pidMoveRotate(angleRef, currentRotation, current_time - last_time, -maxSpeed, maxSpeed, 10, 0.01)
        last_time = current_time

        twistData.angular.z = speedW
        velPub.publish(twistData)
        print(f"speed: {speedW:.2f} sp angle: {angleRef:.2f} Rotation: {currentRotation:.2f}")

        if abs(currentRotation - angleRef) < 0.5:
            if time.time() - pidTime > timeOut:
                break
        else:
            pidTime = time.time()

        time.sleep(0.02)

def setOrientation(angleRef, maxSpeed = 1, timeOut = 5):
    last_time = time.time()
    velPub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    twistData = Twist()
    while not rospy.is_shutdown():
        w,_ = updateOrientation()
        current_time = time.time()
        speedW = pid(angleRef, w, current_time - last_time, -maxSpeed, maxSpeed, 10, 0.01)
        last_time = current_time

        twistData.angular.z = speedW
        velPub.publish(twistData)
        print(f"Speed: {speedW:.2f} Current angle: {w:.2f}")

        if abs(w - angleRef) < 0.5:
            if time.time() - pidTime > timeOut:
                break
        else:
            pidTime = time.time()

        time.sleep(0.02)

state = 0

while not rospy.is_shutdown():
    if state == 0:
        print("=== Mode ===")
        print("0: Exit")
        print("1: Set Orientation")
        print("2: Rotate")
        mode = input("Select Mode: ")

        if mode == "0":
            print("Bye...")
            break
        if mode == "1":
            spOrientation = float(input("Orientation (range from -180 to 180): "))
            maxSpeed = float(input("Rotation speed: "))
            state = 1
        elif mode == "2":
            numberOfRotation = float(input("Number of rotation: "))
            maxSpeed = float(input("Rotation speed: "))
            state = 2

    elif state == 1:
        setOrientation(spOrientation, maxSpeed)
        print("Done!!!\n")
        state = 0

    elif state == 2:
        moveRotate(numberOfRotation*360, maxSpeed)
        print("Done!!!\n")
        state = 0


    # if mode == 0: 
    #     setOrientation(rotationSp)
    # elif mode == 1:
    #     moveRotate(rotationSp)
    # else:
    #     print("arg0: mode, arg1: reference angle")
    #     print("mode option:")
    #     print("0: set orientation")
    #     print("1: rotate with specific angle")
    #     break