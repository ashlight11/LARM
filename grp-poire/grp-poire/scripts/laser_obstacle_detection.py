import math, rospy, numpy,time

from numpy.lib.index_tricks import _fill_diagonal_dispatcher
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import PointCloud2

# DEFINE CONSTANTS
laserData = LaserScan()
commands = Twist()
point_2D = PointCloud2()


if __name__ == '__main__':
    try:
        # Initialize ROS::node
        rospy.init_node('Move_And_Scan', anonymous=True)
        rospy.Subscriber("scan" , LaserScan , laserCallback)
        rospy.Timer(rospy.Duration(0.1), move_command, oneshot = False)
        print("Start move.py")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass