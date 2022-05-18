import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
import numpy as np

numberOfPoseArray = 0
needCalibratePositionStatus = 0
covarianceSum = 0
covarianceTh = 0

def rms(data):
    for i in range(len(data)):
        if(data[i] < 0):
            data[i] = -data[i]
    
    return np.mean(data)

def callbackAMCLUpdate(msgs:PoseWithCovarianceStamped):
    global needCalibratePositionStatus
    global covarianceSum

    covariance = np.array(msgs.pose.covariance)
    covarianceSum = rms(covariance)
    needCalibratePositionStatus = covarianceSum > covarianceTh
    
def callbackParticlecloudUpdate(msgs:PoseArray):
    global numberOfPoseArray
    numberOfPoseArray = len(msgs.poses)

rospy.init_node("check_amcl")
covarianceTh = rospy.get_param("~covariance_th")
rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callbackAMCLUpdate)
rospy.Subscriber("particlecloud", PoseArray, callbackParticlecloudUpdate)

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    print(f"th:{covarianceTh} rms:{covarianceSum} npa: {numberOfPoseArray} status:{needCalibratePositionStatus}")
    rate.sleep()