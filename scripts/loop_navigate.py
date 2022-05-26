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

with open(filePath, 'r') as csvFile:
    csvData = csv.reader(csvFile, delimiter = ',')
    waypointList = np.asarray(list(csvData), dtype = float)
    print("\nWaypoint data:")
    print(waypointList)

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

        reinitPose(0.1, 0.1)
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

def setRobotSpeed(linear, rotation):
    cmdVelPub = rospy.Publisher("cmd_vel", Twist, queue_size=1000)
    data = Twist()

    data.linear.x = linear
    data.angular.z = rotation

    cmdVelPub.publish(data)

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

rospy.init_node("loop_navigate")
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
            currentwayPointGoal = waypointList[currentWaypointGoalIndex]
            currentCalibrationOnGoal = int(currentwayPointGoal[3])
            currentSanitizingOnGoal = int(currentwayPointGoal[4])
            if len(currentwayPointGoal) != 0:
                setGoal(movebaseClient,
                        currentwayPointGoal[0],
                        currentwayPointGoal[1],
                        currentwayPointGoal[2])

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
        elif(key == '0' or key == '\x03'):
            print("Bye.....")
            break

    elif stateNav == 1: # navigating to the goal
        if goalState == 3 and time.time() - lastTime > 5:
            stateNav = 100 # do sanitize
        elif goalState == 4: # goal can't be reached
            print("The goal can't be reached.. Go to next goal")
            currentWaypointGoalIndex = (currentWaypointGoalIndex + 1)%len(waypointList)
            currentwayPointGoal = waypointList[currentWaypointGoalIndex]
            currentCalibrationOnGoal = int(currentwayPointGoal[3])
            currentSanitizingOnGoal = int(currentwayPointGoal[4])
            print("Current waypoint on: <%d> %.2f %.2f %.2f" % (currentWaypointGoalIndex, 
                                                                currentwayPointGoal[0],
                                                                currentwayPointGoal[1],
                                                                currentwayPointGoal[2]))

            setGoal(movebaseClient,
                    currentwayPointGoal[0],
                    currentwayPointGoal[1],
                    currentwayPointGoal[2])
            
            stateNav = 10
            lastTime = time.time()

    elif stateNav == 10: # loop several in this state to chcek if robot really receive the goal target
        if time.time() - lastTime < 2 and goalState == 3:
            print("set goal again")
            setGoal(movebaseClient,
                    currentwayPointGoal[0],
                    currentwayPointGoal[1],
                    currentwayPointGoal[2])

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

        currentWaypointGoalIndex = (currentWaypointGoalIndex + 1)%len(waypointList)
        currentwayPointGoal = waypointList[currentWaypointGoalIndex]
        currentCalibrationOnGoal = int(currentwayPointGoal[3])
        currentSanitizingOnGoal = int(currentwayPointGoal[4])
        print("Current waypoint on: <%d> %.2f %.2f %.2f" % (currentWaypointGoalIndex, 
                                                            currentwayPointGoal[0],
                                                            currentwayPointGoal[1],
                                                            currentwayPointGoal[2]))

        setGoal(movebaseClient,
                currentwayPointGoal[0],
                currentwayPointGoal[1],
                currentwayPointGoal[2])

        lastTime = time.time()
        stateNav = 10

    time.sleep(0.1)