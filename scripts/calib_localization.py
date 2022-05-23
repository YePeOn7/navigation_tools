from email import header
import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import time
import actionlib

covariance = 0

def absSUm(data):
    for i in range(len(data)):
        if(data[i] < 0):
            data[i] = -data[i]
    
    return np.mean(data)

def callbackGoalUpdated(msgs):
    print(msgs)

def callbackAmclPose(msgs:PoseWithCovarianceStamped):
    global covariance

    # rospy.loginfo("AMCL Pose is updated")
    covarience = np.array(msgs.pose.covariance)
    print(f"th:{covariance_th} rms:{absSUm(covarience)}")
    
    if covariance > covariance_th:
        print("need calib")
    # print(type(msgs.pose.covariance))

def setGoalByPublishToTopic(x, y, yaw): # after test, we need to post several time until it is really published to the topic
    pubGoal = rospy.Publisher("move_base/goal", MoveBaseActionGoal, queue_size=10)

    goal = MoveBaseActionGoal()
    goal.header.stamp = rospy.Time.now()
    goal.goal.target_pose.header.frame_id = "map"
    goal.goal.target_pose.header.stamp = rospy.Time.now()
    goal.goal.target_pose.pose.position.x = x
    goal.goal.target_pose.pose.position.y = y
    q = quaternion_from_euler(0,0,math.radians(yaw))
    goal.goal.target_pose.pose.orientation.x = q[0]
    goal.goal.target_pose.pose.orientation.y = q[1]
    goal.goal.target_pose.pose.orientation.z = q[2]
    goal.goal.target_pose.pose.orientation.w = q[3]

    # print(goal)

    pubGoal.publish(goal)

def setGoalByActionlib(client:actionlib.SimpleActionClient, x, y, yaw):
    t = time.time()
    client.wait_for_server()
    print(f"time to wait server: {time.time() - t}")

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

    client.send_goal(goal)
    # start_time = time.time()
    # while not rospy.is_shutdown():
    #     print(client.get_goal_status_text())
    #     time.sleep(0.1)
    #     if(time.time() - start_time > 5):
    #         client.cancel_goal()
    #         print("goal canceled")
    #         break

def movebaseClient(x, y):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id="map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    # start_time = time.time()
    # while not rospy.is_shutdown():
    #     print(client.get_goal_status_text())
    #     time.sleep(0.1)
    #     if(time.time() - start_time > 5):
    #         client.cancel_goal()
    #         print("goal canceled")
    #         break

rospy.init_node("calib_localization")
covariance_th = rospy.get_param("~covariance_th", 0.2)
rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callbackAmclPose)
# rospy.Subscriber("move_base/goal", MoveBaseActionGoal, callbackGoalUpdated)
movebaseClient = actionlib.SimpleActionClient("move_base", MoveBaseAction)
setGoalByActionlib(movebaseClient, 4,1,90)
# time.sleep(1)
# setGoalByPublishToTopic(0,1,0)

while not rospy.is_shutdown():
    # setGoalByPublishToTopic(4,1,0)
    # setGoalByPublishToTopic(-2,1,0)
    # print("test")
    time.sleep(1)