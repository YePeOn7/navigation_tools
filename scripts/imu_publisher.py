#!/usr/bin/env python
import math

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler, euler_from_quaternion

YAW_TOPIC = "/yaw"
FRAME_ID = "imu_link"

def callbackYaw(msg):
    print(msg.data)
    q = quaternion_from_euler(0,0,-math.radians(msg.data))
    pubImu = rospy.Publisher("mcu_imu", Imu, queue_size=100)
    imuData = Imu()

    imuData.header.stamp = rospy.Time.now()
    imuData.header.frame_id = FRAME_ID
    imuData.orientation.x = q[0]
    imuData.orientation.y = q[1]
    imuData.orientation.z = q[2]
    imuData.orientation.w = q[3]
    pubImu.publish(imuData)

if __name__ == "__main__":
    rospy.init_node("imu_publisher")
    rospy.Subscriber("yaw", Float32, callbackYaw)
    rospy.spin()