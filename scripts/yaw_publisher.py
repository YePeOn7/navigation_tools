#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

YAW_TOPIC = "yaw"
yawData = 0

# if __name__ == "__main__":
rospy.init_node("yaw_publisher")
pubYaw = rospy.Publisher(YAW_TOPIC, Float32, queue_size=100)
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    yawData += 0.1
    if yawData > 180:
        yawData -= 360
    print(yawData)

    yawMessage = Float32()
    yawMessage.data = yawData
    pubYaw.publish(yawMessage)
    rate.sleep()