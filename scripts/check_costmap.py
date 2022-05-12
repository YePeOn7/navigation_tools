import rospy
from std_srvs.srv import Empty

rospy.init_node("check_costmap")

clearCostmapService = rospy.ServiceProxy("move_base/clear_costmaps", Empty)
x = clearCostmapService()
print(x)

rospy.spin()