#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
import actionlib
import time
import sys, select, termios, tty


class Rosfun():
    def __init__(self, tfMap = "/map", tfRobotBase = "base_footprint"):
        self.clearCostmapService = rospy.ServiceProxy("move_base/clear_costmaps", Empty)
        self.AMCLNoMotionUpdateService = rospy.ServiceProxy("request_nomotion_update", Empty)
        
        self.movebaseClient = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.goal = MoveBaseGoal()

        rospy.wait_for_service("request_nomotion_update")
        rospy.wait_for_service("move_base/clear_costmaps")

        self.cmdVelPub = rospy.Publisher("cmd_vel", Twist, queue_size=100)
        self.cmdVelData = Twist()
        while self.cmdVelPub.get_num_connections() == 0: # wait the connenction ready
            pass

        self.poseMsg = PoseWithCovarianceStamped()
        self.pubInitialPose = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1000)

        self.tfListener = tf.TransformListener()
        self.mapFrame = tfMap
        self.robotBaseFrame = tfRobotBase
    
    def setRobotSpeed(self, linear, rotation):
        self.cmdVelData.linear.x = linear
        self.cmdVelData.angular.z = rotation
        self.cmdVelPub.publish(self.cmdVelData)

    def clearCostmap(self):
        try:
            self.clearCostmapService()
            return 1
        except rospy.ServiceException as e:
            rospy.loginfo("request_nomotion_update fail: %s" % e)
            return 0

    def amclNoMotionUpdate(self):
        try:
            self.AMCLNoMotionUpdateService()
            return 1
        except rospy.ServiceException as e:
            rospy.loginfo("request_nomotion_update fail: %s" % e)
            return 0

    def getGoalStatus(self):
        return self.movebaseClient.get_state()

    def setGoal(self, x, y, yaw):
        self.goal.target_pose.header.frame_id="map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0,0,yaw)
        self.goal.target_pose.pose.orientation.x = q[0]
        self.goal.target_pose.pose.orientation.y = q[1]
        self.goal.target_pose.pose.orientation.z = q[2]
        self.goal.target_pose.pose.orientation.w = q[3]

        self.movebaseClient.wait_for_server()
        self.movebaseClient.send_goal(self.goal)

    def cancelGoal(self):
        self.movebaseClient.wait_for_server()
        self.movebaseClient.cancel_all_goals()

    def getTfTransformPose2D(self, target, source, maxRetry = 50):
        retry = 0
        while 1:
            try:
                (l, r) = self.tfListener.lookupTransform(target, source, rospy.Time(0))
                tfX = l[0]
                tfY = l[1]
                tfW = euler_from_quaternion(r)[2]
                return [tfX, tfY, tfW]

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("fail to get TF transfor")
                retry+=1
                time.sleep(0.1)

            if retry > maxRetry:
                return [None, None, None]

    def getDistanceFromGoal(self):
        [x, y, _] = self.getTfTransformPose2D(self.mapFrame, self.robotBaseFrame)
        distance = ((self.goal.target_pose.pose.position.x - x)**2 
                    + (self.goal.target_pose.pose.position.y - y)**2)**0.5
        return distance

    def amclSpreadPoseArray(self, covLin, covRot):
        covariance = [covLin, 0.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, covLin, 0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.0, 0.0, 0.0, 0.0, covRot]

        [x, y, w] = self.getTfTransformPose2D(self.mapFrame, self.robotBaseFrame)
        q = quaternion_from_euler(0,0,w)

        self.poseMsg.header.frame_id = self.mapFrame
        self.poseMsg.header.stamp = rospy.Time.now()
        self.poseMsg.pose.pose.position.x = x
        self.poseMsg.pose.pose.position.y = y
        self.poseMsg.pose.pose.position.z = 0
        self.poseMsg.pose.pose.orientation.x = q[0]
        self.poseMsg.pose.pose.orientation.y = q[1]
        self.poseMsg.pose.pose.orientation.z = q[2]
        self.poseMsg.pose.pose.orientation.w = q[3]
        self.poseMsg.pose.covariance = covariance

        self.pubInitialPose.publish(self.poseMsg)  

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

def showOption():
    print("\n****** Option *******")
    print("0: Exit")
    print("1: Clear Costmap")
    print("2: Set Goal")
    print("3: Cancel Goal")
    print("4: Spread AMCL Pose Array")
    print("5: AMCL No Motion Update")

def main():
    rospy.init_node("rosfun")
    rf = Rosfun()
    showOption()
    while not rospy.is_shutdown():
        key = getKey(None)

        if key == '\x03' or key == '0':
            break

        elif key == '1':
            rf.clearCostmap()
        
        elif key == '2':
            goalX = int(input("goal x: "))
            goalY = int(input("goal y: "))
            goalW = int(input("goal w: "))
            rf.setGoal(goalX, goalY, goalW)

        elif key == '3': #Cancel goal
            rf.cancelGoal()
        
        elif key == '4': # spread AMCL pose array
            covLin = float(input("Linear covariance: "))
            covAng = float(input("Angular covariance: "))
            rf.amclSpreadPoseArray(covLin, covAng)

        elif key == '5':
            rf.amclNoMotionUpdate()

        showOption()

if __name__ == "__main__":
    main()