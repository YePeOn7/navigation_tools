#!/usr/bin/env python
import math

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import rospy


YAW_TOPIC = "yaw"

def callbackYaw(msg:Float32):
    q = quaternion_from_euler(0,0,math.radians(msg.data))

    pubImu = rospy.Publisher("mcu_imu", Imu, queue_size=100)
    imuData = Imu()

    imuData.orientation.x = q[0]
    imuData.orientation.y = q[1]
    imuData.orientation.z = q[2]
    imuData.orientation.w = q[3]
    pubImu.publish(imuData)

if __name__ == "__main__":
    rospy.init_node("imu_publisher")
    rospy.Subscriber(YAW_TOPIC, Imu, callbackYaw)
    
    rospy.spin()