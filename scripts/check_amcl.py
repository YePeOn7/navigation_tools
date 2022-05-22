import time
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
import numpy as np
import sys, select, termios, tty
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv import Empty
import dynamic_reconfigure.client

import math

rospy.init_node("check_amcl")
numberOfPoseArray = 0
needCalibratePositionStatus = 0
covarianceSum = 0
covarianceTh = 0
amclPose = PoseWithCovarianceStamped()
amclPose2D = [0,0,0]
tfListener = tf.TransformListener()
referenceFrame = "map"
robotBaseFrame = "base_footprint"
dynamicReconfigureClient = dynamic_reconfigure.client.Client("amcl")

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def rms(data):
    for i in range(len(data)):
        if(data[i] < 0):
            data[i] = -data[i]
    
    return np.mean(data)

def clearCostmap():
    clearCostmapService = rospy.ServiceProxy("move_base/clear_costmaps", Empty)
    clearCostmapService()

def callbackAMCLUpdate(msgs:PoseWithCovarianceStamped):
    global needCalibratePositionStatus
    global covarianceSum
    global amclPose
    global amclPose2D

    amclPose = msgs
    q = [amclPose.pose.pose.orientation.x, 
            amclPose.pose.pose.orientation.y,
            amclPose.pose.pose.orientation.z,
            amclPose.pose.pose.orientation.w]

    amclPose2D = [amclPose.pose.pose.position.x,
                    amclPose.pose.pose.position.y,
                    math.degrees(euler_from_quaternion(q)[2])]

    covariance = np.array(msgs.pose.covariance)
    covarianceSum = rms(covariance)
    needCalibratePositionStatus = covarianceSum > covarianceTh
    
def callbackParticlecloudUpdate(msgs:PoseArray):
    global numberOfPoseArray
    numberOfPoseArray = len(msgs.poses)
  
def getTfTransformPose2D(target, source):
    tfX = tfY = tfW = 0

    while 1:
        try:
            (l, r) = tfListener.lookupTransform(target, source, rospy.Time(0))
            tfX = l[0]
            tfY = l[1]
            tfW = math.degrees(euler_from_quaternion(r)[2])
            return [tfX, tfY, tfW]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("fail to get TF transfor")

def amclNoMotionUpdate():
    rospy.wait_for_service('request_nomotion_update')
    try:
        AMCLNoMotionUpdateService = rospy.ServiceProxy("request_nomotion_update", Empty)
        AMCLNoMotionUpdateService()
    except:
        print("request_nomotion_update fail!!")

def reinitPose(covLin, covRot):
    covariance = [covLin, 0.0, 0.0, 0.0, 0.0, 0.0, 
                    0.0, covLin, 0.0, 0.0, 0.0, 0.0, 
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                    0.0, 0.0, 0.0, 0.0, 0.0, covRot]

    [x, y, w] = getTfTransformPose2D(referenceFrame, robotBaseFrame)
    q = quaternion_from_euler(0,0,math.radians(w))

    poseMsg = PoseWithCovarianceStamped()
    poseMsg.header.frame_id = "map"
    poseMsg.header.stamp = rospy.Time.now()
    poseMsg.pose.pose.position.x = x
    poseMsg.pose.pose.position.y = y
    poseMsg.pose.pose.position.z = 0
    poseMsg.pose.pose.orientation.x = q[0]
    poseMsg.pose.pose.orientation.y = q[1]
    poseMsg.pose.pose.orientation.z = q[2]
    poseMsg.pose.pose.orientation.w = q[3]
    poseMsg.pose.covariance = covariance

    pubInitialPose.publish(poseMsg)    

def amclVsTf():
    print("press q to quit....")
    time.sleep(2)
    while not rospy.is_shutdown():
        key = getKey(0.1)
        if key == 'q':
            break
        [tfX, tfY, tfW] = getTfTransformPose2D(referenceFrame, robotBaseFrame)

        print("AMCL:(%.2f, %.2f, %.2f) TF:(%.2f, %.2f, %.2f)"%(amclPose2D[0], amclPose2D[1], amclPose2D[2], tfX, tfY, tfW))

def showOption():
    print("======= Option ========")
    print("0: exit")
    print("1: amcl update with rotation covariance 0")
    print("2: amcl update with rotation covariance 0.5")
    print("3: amcl update with custom covariance")
    print("4: monitorting amcl vs map->base_link transform")
    print("5: Single No Motion Update")
    print("6: Loop No Motion Update")
    print("7: Update alpha1 and alpha2: 0.1")
    print("8: Update alpha1 and alpha2: 0.0")
    print("9: Clear costmap")

covarianceTh = rospy.get_param("~covariance_th", 0.2)
pubInitialPose = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1000)
rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callbackAMCLUpdate)
rospy.Subscriber("particlecloud", PoseArray, callbackParticlecloudUpdate)

rate = rospy.Rate(1)
settings = termios.tcgetattr(sys.stdin)
showOption()

# while not rospy.is_shutdown():
#     # getTfTransformPose2D(referenceFrame, robotBaseFrame)
#     amclVsTf()

# exit(0)

while not rospy.is_shutdown():
    key = getKey(None)
    # print(key)

    if(key == '1'):
        print("Updating initpose with covariannce linear: 1.5 Rotation: 0")
        reinitPose(1.5, 0)
    elif(key == '2'):
        print("Updating initpose with covariannce linear: 1.5 Rotation: 0.5")
        reinitPose(1.5, 0.5)
    elif(key == '3'):
        covLin = float(input("Covariance linear: "))
        covRad = float(input("Covariance Rotation: "))
        print("Updating initpose with covariannce linear: %.1f Rotation: %.1f"%(covLin, covRad))
        reinitPose(covLin, covRad)
    elif(key == "4"):
        amclVsTf()
    elif(key == "5"):
        print("Peforming AMCL no motion Update......")
        amclNoMotionUpdate()
    elif(key == "6"):
        loopNumber = int(input("Number of loops: "))
        for i in range(loopNumber):
            print("peform AMCL No motion Update %d"%(i))
            amclNoMotionUpdate()
            time.sleep(0.1)
    elif(key == '7'):
        params  = {"odom_alpha1": 0.1, "odom_alpha2": 0.1}
        dynamicReconfigureClient.update_configuration(params)
        print("odom_alpha1 and odom_alpha2 have been updated to 0.1")
    elif(key == '8'):
        params  = {"odom_alpha1": 0.0, "odom_alpha2": 0.0}
        dynamicReconfigureClient.update_configuration(params)
        print("odom_alpha1 and odom_alpha2 have been updated to 0.0")
    elif(key == "9"):
        clearCostmap()
        
    elif (key == '\x03' or '0'):
        print("byeee....")
        # print(f"{key!r}")
        break

    showOption()