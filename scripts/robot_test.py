from numpy import rate
import sys
import rospy
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
import math
import time

rospy.init_node("robot_test")
referenceFrame = "/odom"
robotBaseFrame = "/base_footprint"
robotStartFrame = "/start_frame"

orientation = 0
tfListener = tf.TransformListener()
tfBroadcaster = tf.TransformBroadcaster()

last_angle = 0
error_angle = 0
kp_rotation = 0.05
ki_rotation = 0.005
kd_rotation = 0.01
I_rotation = 0
last_error_rotation = 0

last_linear = 0
error_linear = 0
kp_linear = 0.5
ki_linear = 0.01
kd_linear = 0.1
I_linear = 0
last_error_linear = 0

startFrameLinear = 0
startFrameQuaternion = 0

pidTime = 0

def setSpeed(linearSpeed, rotationSpeed):
    velPub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    twistData = Twist()
    twistData.linear.x = linearSpeed
    twistData.angular.z = rotationSpeed
    velPub.publish(twistData)

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

def pidRotation(sp, current, dt, min, max, th_I_top, th_I_bottom):
    global I_rotation
    global last_error_rotation

    I_limiter_gain = 0.8
    error = getAngleError(sp, current)

    P = kp_rotation * error
    if abs(error) < th_I_top and abs(error) > th_I_bottom:
        I_rotation += ki_rotation * error * dt
    else:
        I_rotation = 0
    d = kd_rotation * (error - last_error_rotation) 

    if I_rotation > I_limiter_gain * max: I_rotation = I_limiter_gain * max
    elif I_rotation < I_limiter_gain * min: I_rotation = I_limiter_gain * min

    PID = P + I_rotation + d

    if PID > max: PID = max
    elif PID < min: PID = min

    last_error_rotation = error

    return PID

def pidMoveRotate(sp, current, dt, min, max, th_I_top, th_I_bottom):
    global I_rotation
    global last_error_rotation

    I_limiter_gain = 0.8

    error = sp - current

    P = kp_rotation * error
    if abs(error) < th_I_top and abs(error) > th_I_bottom:
        I_rotation += ki_rotation * error * dt
    else:
        I_rotation = 0
    d = kd_rotation * (error - last_error_rotation) 

    if I_rotation > I_limiter_gain * max: I_rotation = I_limiter_gain * max
    elif I_rotation < I_limiter_gain * min: I_rotation = I_limiter_gain * min

    PID = P + I_rotation + d

    if PID > max: PID = max
    elif PID < min: PID = min

    last_error_rotation = error

    return PID

def pidLinear(sp, current, dt, min, max, th_I_top, th_I_bottom):
    global I_linear
    global last_error_linear

    I_limiter_gain = 0.01

    error = sp - current

    P = kp_linear * error
    if abs(error) < th_I_top and abs(error) > th_I_bottom:
        I_linear += ki_linear * error * dt
    else:
        I_linear = 0
    D = kd_linear * (error - last_error_linear) 

    if I_linear > I_limiter_gain * max: I_linear = I_limiter_gain * max
    elif I_linear < I_limiter_gain * min: I_linear = I_limiter_gain * min

    PID = P + I_linear + D

    if PID > max: PID = max
    elif PID < min: PID = min

    last_error_linear = error

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
        speedW = pidRotation(angleRef, w, current_time - last_time, -maxSpeed, maxSpeed, 10, 0.01)
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

def getStartFrameTransform(timeOut = 5):
    global startFrameLinear
    global startFrameQuaternion

    lastTime = time.time()
    while not rospy.is_shutdown():
        try:
            (startFrameLinear, startFrameQuaternion) = tfListener.lookupTransform(referenceFrame, robotBaseFrame, rospy.Time(0))
            tfBroadcaster.sendTransform(startFrameLinear, startFrameQuaternion, rospy.Time.now(), robotStartFrame, referenceFrame)
            print("start frame has been initialized")
            return 0

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Initializing linear transform")

        if time.time() - lastTime > timeOut:
            print("failed to init linear transform")
            return -1

def updateStartFrame():
    global startFrameQuaternion
    global startFrameLinear
    
    try:
        tfBroadcaster.sendTransform(startFrameLinear, startFrameQuaternion, rospy.Time.now(), robotStartFrame, referenceFrame)
        return 0

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("update start frame fail")

def updateLinear():
    try:
        (l,_) = tfListener.lookupTransform(robotStartFrame, robotBaseFrame, rospy.Time(0))
        x = l[0]
        # print(l)
        return x
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
    
    return 0

def moveLinear(distance, maxSpeed = 0.5, timeOut = 5, tolerance = 0.02):
    tfTime = lastTime = time.time()
    getStartFrameTransform()
    while not rospy.is_shutdown():
        if time.time() - tfTime > 0.01:
            updateStartFrame()
            tfTime = time.time()

        x = updateLinear()
        currentTime = time.time()
        speedX = pidLinear(distance, x, currentTime - lastTime, -maxSpeed, maxSpeed, 0.05, 0.01)
        setSpeed(speedX, 0)
        print(f"target: {distance} current x: {x:.2f} speed:{speedX:.2f}")

        if abs(x - distance) < tolerance:
            if time.time() - pidTime > timeOut:
                I_linear = 0
                break
        else:
            pidTime = time.time()

        time.sleep(0.001)

state = 0
# while not rospy.is_shutdown():
#     initLinearTransform()

# exit(0)

def moveLinear2(distance, maxSpeed = 0.5, acc = 0.1, decc = 0.1, timeOut = 5, tolerance = 0.02):
    tfTime = lastTime = time.time()
    getStartFrameTransform()
    speed = 0
    stateAcc = 0 #0 acc, 1 decc
    while not rospy.is_shutdown():
        if time.time() - tfTime > 0.01:
            updateStartFrame()
            tfTime = time.time()

        if not stateAcc:
            speed += min(acc * 0.001, maxSpeed - speed)
            if speed >= maxSpeed:
                stateAcc = 1
        else:
            speed -= min(decc * 0.001, speed)
            if speed <= 0:
                stateAcc = 0

        x = updateLinear()
        currentTime = time.time()
        speedX = pidLinear(distance, x, currentTime - lastTime, -speed, speed, 0.05, 0.01)
        setSpeed(speedX, 0)
        print("target: %.2f current x: %.2f speed:%.2f" % (distance,x,speedX))
        # print("speed: %.2f max: %.2f state: %d" % (speed, maxSpeed, stateAcc))

        if abs(x - distance) < tolerance:
            if time.time() - pidTime > timeOut:
                I_linear = 0
                break
        else:
            pidTime = time.time()

        time.sleep(0.001)

state = 0
# while not rospy.is_shutdown():
#     initLinearTransform()

# exit(0)

while not rospy.is_shutdown():
    if state == 0:
        print("=== Mode ===")
        print("0: Exit")
        print("1: Set Orientation")
        print("2: Rotate")
        print("3: Set Linear Movement")
        print("4: Set Linear Movementwith Acc/Dec")
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
        elif mode == "3":
            distanceSp = float(input("Distance: "))
            maxSpeed = float(input("Linear Speed: "))
            state = 3
        elif mode == "4":
            distanceSp = float(input("Distance: "))
            maxSpeed = float(input("Linear Speed: "))
            acc = float(input("Acceleration: "))
            decc = float(input("Deceleration: "))
            state = 4

    elif state == 1:
        setOrientation(spOrientation, maxSpeed)
        print("Done!!!\n")
        state = 0

    elif state == 2:
        moveRotate(numberOfRotation*360, maxSpeed)
        print("Done!!!\n")
        state = 0

    elif state == 3:
        moveLinear(distanceSp, maxSpeed)
        print("Done!!")
        state = 0

    elif state == 4:
        moveLinear2(distanceSp, maxSpeed, acc, decc)
        print("Done!!")
        state = 0