#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import actionlib

import time
import sys, select, termios, tty

class base():
    def __init__(self):
        pass

class Rosfun():
    def __init__(self, nodeName):
        rospy.init_node(nodeName)
        self.base = base()

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
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id="map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0,0,yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        # t = time.time()
        self.movebaseClient.wait_for_server()
        # print(f"time to wait server: {time.time() - t}")

        self.movebaseClient.send_goal(goal)
        # status = self.getGoalStatus()
        # while not (status == 0 or status == 1):
        #     self.movebaseClient.send_goal(goal)
        #     status = self.getGoalStatus(self.movebaseClient)

    def cancelGoal(self):
        self.movebaseClient.wait_for_server()
        self.movebaseClient.cancel_all_goals()

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
    print("****** Option *******")
    print("0: Exit")
    print("1: Clear Costmap")
    print("2: Set Goal")
    print("3: Check goal status")

def main():
    rf = Rosfun("rosfun")
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

        elif key == '3':
            print("Press q to close...")
            while 1:
                st = rf.getGoalStatus()
                print("goal status: %d"%st)
                if getKey(0.5) == 'q':
                    break

        showOption()

if __name__ == "__main__":
    main()