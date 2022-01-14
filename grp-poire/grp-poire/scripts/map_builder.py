import rospy
from nav_msgs.msg import OccupancyGrid

def mapCallback(data):
    
    print("a")

if __name__ == '__main__':
    try:
        # Initialize ROS::node
        rospy.init_node('move', anonymous=True)
        rospy.Subscriber("/scan" , OccupancyGrid , mapCallback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass