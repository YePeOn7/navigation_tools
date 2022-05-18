import rospy
import csv
import os
import numpy as np
import time
import actionlib
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Twist
import math

# csvReader = csv.reader()
fileDirPath = os.path.dirname(__file__) + "/../waypoints"
filePath = f"{fileDirPath}/waypoints.txt"
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

with open(filePath, 'r') as csvFile:
    csvData = csv.reader(csvFile, delimiter = ',')
    waypointList = np.asarray(list(csvData), dtype = float)
    print(waypointList)

def rms(array:list):
    array = np.array(array)
    rms = (np.sum(array**2)/len(array))**0.5

def callbackAMCLUpdate(msgs:PoseWithCovarianceStamped):
    global needCalibratePositionStatus
    global covarianceSum

    covariance = np.array(msgs.pose.covariance)
    covarianceSum = rms(covariance)
    needCalibratePositionStatus = covarianceSum > covarianceTh
    
def callbackParticlecloudUpdate(msgs:PoseArray):
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

def getGoalStatus(client:actionlib.SimpleActionClient):
    return client.get_state()

def setGoal(client:actionlib.SimpleActionClient, x, y, yaw):
    goal = generateGoal(x,y,yaw)

    # t = time.time()
    client.wait_for_server()
    # print(f"time to wait server: {time.time() - t}")

    client.send_goal(goal)
    status = getGoalStatus(client)
    while not (status == 0 or status == 1):
        client.send_goal(goal)
        status = getGoalStatus(client)

def cancelGoal(client:actionlib.SimpleActionClient):
    client.wait_for_server()
    client.cancel_all_goals()

def setRobotSpeed(linear, rotation):
    cmdVelPub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
    data = Twist()

    data.linear.x = linear
    data.angular.z = rotation

    cmdVelPub.publish(data)

def sanitize():
    print("sanitizing.......")
    setRobotSpeed(0,1)
    time.sleep(2)
    setRobotSpeed(0,-1)
    time.sleep(4)
    

rospy.init_node("loop_navigate")
movebaseClient = actionlib.SimpleActionClient("move_base", MoveBaseAction)

lastTime = 0
# cancelGoal(movebaseClient)
# exit(0)
while not rospy.is_shutdown():
    movebaseClient.wait_for_server()
    goalState = getGoalStatus(movebaseClient)
    # print(goalState, currentWaypointGoalIndex)

    if stateNav == 0:
        currentwayPointGoal = waypointList[currentWaypointGoalIndex]
        if len(currentwayPointGoal) != 0:
            setGoal(movebaseClient,
                    currentwayPointGoal[0],
                    currentwayPointGoal[1],
                    currentwayPointGoal[2])

            lastTime = time.time()
            stateNav = 1

    elif stateNav == 1: # increase index waypoint and set goal
        if goalState == 3 and time.time() - lastTime > 5:
            stateNav = 100 # do sanitize

    elif stateNav == 10: # loop several in this state to chcek if robot really receive the goal target
        if time.time() - lastTime < 2 and goalState == 3:
            print("set goal again")
            setGoal(movebaseClient,
                    currentwayPointGoal[0],
                    currentwayPointGoal[1],
                    currentwayPointGoal[2])

        elif time.time() - lastTime > 2:
            stateNav = 1

    elif stateNav == 100:
        sanitize()

        currentWaypointGoalIndex = (currentWaypointGoalIndex + 1)%len(waypointList)
        currentwayPointGoal = waypointList[currentWaypointGoalIndex]
        print(f"next waypoint on:<{currentWaypointGoalIndex}> {currentwayPointGoal}")

        setGoal(movebaseClient,
                currentwayPointGoal[0],
                currentwayPointGoal[1],
                currentwayPointGoal[2])

        lastTime = time.time()
        stateNav = 10

    time.sleep(0.1)