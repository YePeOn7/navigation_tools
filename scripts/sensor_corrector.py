#!/usr/bin/env python
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Imu
import math

import rospy

DEBUG = False
IMU_TOPIC = "imu/data"
CORRECTED_IMU_TOPIC = "corrected_imu"
ENCODER_TOPIC = "encoder"

IMU_YAW_CORRECTOR_GAIN = -0.01

rotation = 0
previousYaw = 0
correctedYaw = 0

rospy.init_node("sensor_corrector")

def callbackIMU(msg):
    global rotation
    global previousYaw
    global correctedYaw

    IMU_original = msg

    q = (msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w)
    
    ############ fix the yaw ##############
    euler = euler_from_quaternion(q)
    
    yaw = math.degrees(euler[2])
    
    delta = yaw - previousYaw
    if delta > 180:
        delta -=360
    elif delta < -180:
        delta += 360
    
    rotation += delta
    previousYaw = yaw

    temp = (yaw - IMU_YAW_CORRECTOR_GAIN * rotation) % 360

    if temp > 180:
        temp -= 360

    correctedYaw = temp
    correctedYawRadian = math.radians(correctedYaw)
    correctedQuaternion = quaternion_from_euler(euler[0], euler[1], correctedYawRadian)
    if DEBUG:
        print("yaw: %.2f corrected: %.2f rot: %.2f" % (yaw, correctedYaw, rotation))

    IMU_corrected = IMU_original
    IMU_corrected.orientation.x = correctedQuaternion[0]
    IMU_corrected.orientation.y = correctedQuaternion[1]
    IMU_corrected.orientation.z = correctedQuaternion[2]
    IMU_corrected.orientation.w = correctedQuaternion[3]

    pubCorrectedIMU.publish(IMU_corrected)

def callbackRotary():
    pass

if __name__=="__main__":
    pubCorrectedIMU = rospy.Publisher(CORRECTED_IMU_TOPIC, Imu, queue_size=100)
    subIMU = rospy.Subscriber(IMU_TOPIC, Imu, callbackIMU)

    rospy.spin()

   