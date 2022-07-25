import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import math

def imuEulerCallback(msg:Float32MultiArray):
    data = msg.data
    imuMsg = Imu()
    q = quaternion_from_euler(math.radians(data[0]),
                                math.radians(data[1]),
                                math.radians(data[2]))
    
    imuMsg.orientation.x = q[0]
    imuMsg.orientation.y = q[1]
    imuMsg.orientation.z = q[2]
    imuMsg.orientation.w = q[3]
    
    imuMsg.angular_velocity.x=data[3]
    imuMsg.angular_velocity.y=data[4]
    imuMsg.angular_velocity.z=data[5]
    
    imuMsg.linear_acceleration.x=data[6]
    imuMsg.linear_acceleration.y=data[7]
    imuMsg.linear_acceleration.z=data[8]

    imuPub.publish(imuMsg)
    # print(data)

rospy.init_node("imu_publisher")

imuPub = rospy.Publisher("imu", Imu, queue_size=100)
rospy.Subscriber("imuEuler", Float32MultiArray, imuEulerCallback)

if __name__ == "__main__":
    rospy.spin()