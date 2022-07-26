#!/usr/bin/env python3
import rosfun
import rospy
import time
from actionlib_msgs.msg import GoalStatus

rospy.init_node("rosfun_test")
rf = rosfun.Rosfun()

pos = {"x":[-3,-1, -1],
        "y":[1,1,2],
        "w":[0,0,0]}

state = 0

indexPosition = 0

while not rospy.is_shutdown():
    if state == 0:
        rf.setGoal(pos['x'][0], pos['y'][0], pos['w'][0])
        state = 1
        lastTime = time.time()
    elif state == 1:
        time.sleep(0.05)
        
        goalStatus = rf.getGoalStatus()
        distance = rf.getDistanceFromGoal()
        print("index: %d status: %d distance: %.2f"%(indexPosition, goalStatus, distance))
        if goalStatus == GoalStatus.SUCCEEDED:
            indexPosition = (indexPosition +1) % 3
            rf.setGoal(pos['x'][indexPosition], 
                        pos['y'][indexPosition], 
                        pos['w'][indexPosition])