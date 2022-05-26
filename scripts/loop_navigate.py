#!/usr/bin/env python
import rospy
import csv
import os
import numpy as np
import time
import actionlib
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Twist
from std_srvs.srv import Empty
import dynamic_reconfigure.client
import math
import sys, select, termios, tty

CALIBRATION_LOOP_NUMBER = 50
RECOVERY_LINEAR = -0.3   #in m
RECOVERY_ANGULAR = 180  #in degree
RECOVERY_LINEAR_SPEED = 0.3
RECOVERY_ANGULAR_SPEED = 0.3
RECOVERY_MAX_RETRY = 3

fileDirPath = os.path.dirname(__file__) + "/../waypoints"
filePath = fileDirPath+"/waypoints.txt"
waypointList = []
numberOfWayPoint = 0
currentWaypointGoalIndex = 0
currentwayPointGoal = []
goalState = 0
stateNav = 0

numberOfPoseArray = 0
needCalibratePositionStatus = 0
covarianceSum = 0
covarianceTh = 0

referenceFrame = "map"              # use for AMCL update
robotBaseFrame = "robot_footprint"   # use for AMCL update
odomFrame = "odom"
robotStartFrame = "start_frame"    # use when we want to use pid movement

retry = 0

orientation = 0

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

with open(filePath, 'r') as csvFile:
    csvData = csv.reader(csvFile, delimiter = ',')
    waypointList = np.asarray(list(csvData), dtype = float)
    print("\nWaypoint data:")
    print(waypointList)

def setRobotSpeed(linear, rotation):
    cmdVelPub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
    data = Twist()

    data.linear.x = linear
    data.angular.z = rotation

    cmdVelPub.publish(data)

def updateStartFrame():
    global startFrameQuaternion
    global startFrameLinear
    
    try:
        tfBroadcaster.sendTransform(startFrameLinear, startFrameQuaternion, rospy.Time.now(), robotStartFrame, referenceFrame)
        return 0

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("update start frame fail")

def pidRotation(sp, current, dt, min, max, th_I_top, th_I_bottom):      # use for setting orientation
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
        print("Speed: %.2f Current angle: %.2f"%(speedW, w))

        if abs(w - angleRef) < 0.5:
            if time.time() - pidTime > timeOut:
                break
        else:
            pidTime = time.time()

        time.sleep(0.02)

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
    global I_linear
    tfTime = lastTime = time.time()
    getStartFrameTransform()
    while not rospy.is_shutdown():
        if time.time() - tfTime > 0.01:
            updateStartFrame()
            tfTime = time.time()

        x = updateLinear()
        currentTime = time.time()
        speedX = pidLinear(distance, x, currentTime - lastTime, -maxSpeed, maxSpeed, 0.05, 0.01)
        setRobotSpeed(speedX, 0)
        # print("target: %.2f current x: %.2f speed: %.2f"%(distance, x, speedX))

        if abs(x - distance) < tolerance:
            if time.time() - pidTime > timeOut:
                I_linear = 0
                break
        else:
            pidTime = time.time()

        time.sleep(0.001)

def pidMoveRotate(sp, current, dt, min, max, th_I_top, th_I_bottom):    # use for setting rotation
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
        # print("speed: %.2f sp angle: %.2f Rotation: %.2f"%(speedW, angleRef, currentRotation))

        if abs(currentRotation - angleRef) < 0.5:
            if time.time() - pidTime > timeOut:
                break
        else:
            pidTime = time.time()

        time.sleep(0.02)

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def updateOdomAlphaParam(alpha1, alpha2, alpha3, alpha4):
    dynamicReconfigureClient = dynamic_reconfigure.client.Client("amcl")
    params  = {"odom_alpha1": alpha1, 
                "odom_alpha2": alpha2,
                "odom_alpha3": alpha3,
                "odom_alpha4": alpha4}
    dynamicReconfigureClient.update_configuration(params)

def getTfTransformPose2D(target, source):
    tfX = tfY = tfW = 0

    while 1:
        try:
            (l, r) = tfListener.lookupTransform(target, source, rospy.Time(0))
            tfX = l[0]
            tfY = l[1]
            tfW = math.degrees(euler_from_quaternion(r)[2])
            return [tfX, tfY, tfW]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("fail to get TF transfor")

def reinitPose(covLin, covRot):
    covariance = [covLin, 0.0, 0.0, 0.0, 0.0, 0.0, 
                    0.0, covLin, 0.0, 0.0, 0.0, 0.0, 
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                    0.0, 0.0, 0.0, 0.0, 0.0, covRot]

    [x, y, w] = getTfTransformPose2D(referenceFrame, robotBaseFrame)
    q = quaternion_from_euler(0,0,math.radians(w))

    poseMsg = PoseWithCovarianceStamped()
    poseMsg.header.frame_id = "map"
    poseMsg.header.stamp = rospy.Time.now()
    poseMsg.pose.pose.position.x = x
    poseMsg.pose.pose.position.y = y
    poseMsg.pose.pose.position.z = 0
    poseMsg.pose.pose.orientation.x = q[0]
    poseMsg.pose.pose.orientation.y = q[1]
    poseMsg.pose.pose.orientation.z = q[2]
    poseMsg.pose.pose.orientation.w = q[3]
    poseMsg.pose.covariance = covariance

    pubInitialPose = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1000)
    pubInitialPose.publish(poseMsg)  

def amclNoMotionUpdate():
    rospy.wait_for_service('request_nomotion_update')
    try:
        AMCLNoMotionUpdateService = rospy.ServiceProxy("request_nomotion_update", Empty)
        AMCLNoMotionUpdateService()
    except:
        print("request_nomotion_update fail!!")

def changeAlpha(alpha):
    dynamicReconfigureClient = dynamic_reconfigure.client.Client("amcl")
    params  = {"odom_alpha1": alpha, "odom_alpha2": alpha}
    dynamicReconfigureClient.update_configuration(params)

def calibrate(loopNumber, mode = 0):
    print("Calibrating...")
    if mode == 0:
        reinitPose(1.5, 0.5)
        for i in range(loopNumber):
            amclNoMotionUpdate()
            time.sleep(0.1)

        reinitPose(0.2, 0.1)
        for i in range(loopNumber):
            amclNoMotionUpdate()
            time.sleep(0.1)
        reinitPose(0.05, 0.0)
        
def rms(array):
    array = np.array(array)
    rms = (np.sum(array**2)/len(array))**0.5

def callbackAMCLUpdate(msgs):
    global needCalibratePositionStatus
    global covarianceSum

    covariance = np.array(msgs.pose.covariance)
    covarianceSum = rms(covariance)
    needCalibratePositionStatus = covarianceSum > covarianceTh
    
def callbackParticlecloudUpdate(msgs):
    global numberOfPoseArray
    numberOfPoseArray = len(msgs.poses)

def generateGoal(x, y, yaw):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id="map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    q = quaternion_from_euler(0,0,math.radians(yaw))
    goal.target_pose.pose.orientation.x = q[0]
    goal.target_pose.pose.orientation.y = q[1]
    goal.target_pose.pose.orientation.z = q[2]
    goal.target_pose.pose.orientation.w = q[3]

    return goal

def getGoalStatus(client):
    return client.get_state()

def setGoal(client, x, y, yaw):
    goal = generateGoal(x,y,yaw)

    # t = time.time()
    client.wait_for_server()
    # print(f"time to wait server: {time.time() - t}")

    client.send_goal(goal)
    status = getGoalStatus(client)
    while not (status == 0 or status == 1):
        client.send_goal(goal)
        status = getGoalStatus(client)

def cancelGoal(client):
    client.wait_for_server()
    client.cancel_all_goals()

def sanitize():
    print("Sanitizing.......")
    setRobotSpeed(0,0) # need this to make next move directly move
    time.sleep(0.5)
    setRobotSpeed(0,1)
    time.sleep(2)
    setRobotSpeed(0,-1)
    time.sleep(4)
    setRobotSpeed(0,1)
    time.sleep(2)
    setRobotSpeed(0,0)

def clearCostmap():
    clearCostmapService = rospy.ServiceProxy("move_base/clear_costmaps", Empty)
    clearCostmapService()
    
def showOption():
    print("\n<------ Option ------->")
    print("0: Exit")
    print("1: Start navigation")
    print("2: Test sanitizing function")
    print("3: Test recovery behaviour")

def setGoalByIndex(index):
    global currentCalibrationOnGoal
    global currentSanitizingOnGoal
    global currentwayPointGoal

    currentwayPointGoal = waypointList[index]
    currentCalibrationOnGoal = int(currentwayPointGoal[3])
    currentSanitizingOnGoal = int(currentwayPointGoal[4])
    if len(currentwayPointGoal) != 0:
        setGoal(movebaseClient,
                currentwayPointGoal[0],
                currentwayPointGoal[1],
                currentwayPointGoal[2])
        return 0
    return 1 # the way point data is not correct

def nextGoal():
    global currentWaypointGoalIndex
    
    currentWaypointGoalIndex = (currentWaypointGoalIndex+1)%len(waypointList)
    print("Current waypoint on: <%d> %.2f %.2f %.2f" % (currentWaypointGoalIndex, 
                                                            currentwayPointGoal[0],
                                                            currentwayPointGoal[1],
                                                            currentwayPointGoal[2]))
    setGoalByIndex(currentWaypointGoalIndex)


def recoveryBehaviour():
    print("Recovering......")
    moveLinear(RECOVERY_LINEAR, RECOVERY_LINEAR_SPEED, 2, 0.05)
    moveRotate(RECOVERY_ANGULAR, RECOVERY_ANGULAR_SPEED, 2)

rospy.init_node("loop_navigate")
tfListener = tf.TransformListener()
tfBroadcaster = tf.TransformBroadcaster()
movebaseClient = actionlib.SimpleActionClient("move_base", MoveBaseAction)
tfListener = tf.TransformListener()
settings = termios.tcgetattr(sys.stdin)
lastTime = 0
# cancelGoal(movebaseClient)
# exit(0)
while not rospy.is_shutdown():
    movebaseClient.wait_for_server()
    goalState = getGoalStatus(movebaseClient)
    key = getKey(0.01)
    # print(goalState, currentWaypointGoalIndex)

    if key == 'q':
        stateNav = 0
        cancelGoal(movebaseClient)
        setRobotSpeed(0,0)

    if stateNav == 0:   # entering state (only access at the first time the program run)
        showOption()
        key = getKey(None)
        if key == '1':
            clearCostmap()
            if not setGoalByIndex(currentWaypointGoalIndex):
                lastTime = time.time()
                stateNav = 1
                print("\nStart Navigating....")
                print("Press q to exit navigating....")
                print("Current waypoint on: <%d> %.2f %.2f %.2f" % (currentWaypointGoalIndex, 
                                                                    currentwayPointGoal[0],
                                                                    currentwayPointGoal[1],
                                                                    currentwayPointGoal[2]))
        elif key == '2':
            sanitize()
        elif key == '3':
            recoveryBehaviour()
        elif(key == '0' or key == '\x03'):
            print("Bye.....")
            break

    elif stateNav == 1: # navigating to the goal
        if goalState == 3 and time.time() - lastTime > 5:
            stateNav = 100 # do sanitize
            retry = 0

        elif goalState == 4: # goal can't be reached
            retry+=1
            if retry > RECOVERY_MAX_RETRY:
                print("The goal can't be reached.. Go to next goal")
                nextGoal()
            
            else:
                print("Entering recovery mode on retry: %d"%(retry))
                recoveryBehaviour()
                setGoalByIndex(currentWaypointGoalIndex)
            
            stateNav = 10
            lastTime = time.time()

    elif stateNav == 10: # loop several in this state to check if robot really receive the goal target
        if time.time() - lastTime < 2 and goalState == 3:
            print("set goal again")
            setGoalByIndex(currentWaypointGoalIndex)

        elif time.time() - lastTime > 2:
            stateNav = 1

    elif stateNav == 100: #sanitizing and set the next goal after
        if(currentSanitizingOnGoal):
            sanitize()
        else:
            print("No sanitizing will be peformed in this goal location")

        if(currentCalibrationOnGoal):
            calibrate(CALIBRATION_LOOP_NUMBER)
        else:
            print("No calibration will be peformed in this goal location")

        nextGoal()
        lastTime = time.time()
        stateNav = 10

    time.sleep(0.1)