#!/usr/bin/python3
import math, rospy, numpy,time

from numpy.lib.index_tricks import _fill_diagonal_dispatcher
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import PointCloud2

# DEFINE CONSTANTS
laserData = LaserScan()
commands = Twist()
point_2D = PointCloud2()
FORWARD_SPEED_MPS = 1.5
TURN_SPEED_MPS = 1
ANGULAR_TURN = 2

# Callback function for laser data
def laserCallback(data):
    laserData = data
    nb_values = len(laserData.ranges)
     # splitting the right and left sides of the robot
    right = laserData.ranges[:math.floor(nb_values/6)]
    left = laserData.ranges[math.floor(5 * nb_values/6):]

    obstacles = []
    angle = data.angle_min
    for aDistance in data.ranges :
        if 0.1 < aDistance and aDistance < 2.0 :
            aPoint= [ 
                math.cos(angle) * aDistance, 
                math.sin( angle ) * aDistance
            ]
            obstacles.append( aPoint )
        angle+= data.angle_increment
    #rospy.loginfo( str(
        #[[ round(p[0], 2), round(p[1], 2) ] for p in  obstacles[0:10] ] 
    #) + " ..." )
    for point in obstacles : 
        # if point is in a dangerous zone
        if ( point[0] > 0.05 and point[0] < 0.5 and abs(point[1]) < 0.3): 
            point_2D.data = point
            #if the point is really close, speed up and reverse
            if(point[0] < 0.2):
                commands.linear.x = - 2 * TURN_SPEED_MPS
            else : commands.linear.x = TURN_SPEED_MPS
            #if there is more room on the right side, go right
            if ((numpy.amax(right) > numpy.amax(left) and commands.linear.x > 0 )
            or (numpy.amax(right) < numpy.amax(left) and commands.linear.x < 0 )):      
                commands.angular.z = - ANGULAR_TURN
            # else, go left
            else :
                commands.angular.z = ANGULAR_TURN
        # if not dangerous, stay as it is
        else :
            commands.linear.x = FORWARD_SPEED_MPS
            commands.angular.z = 0.0


# publisher for nav commands
commandPublisher = rospy.Publisher(
    '/cmd_vel_mux/input/navi',
    Twist, queue_size=10
)

# publisher for 2D Points
rvizPublisher = rospy.Publisher(
    '/rviz_point_cloud',
    PointCloud2, queue_size=10
)

# publish at rate 10 Hz
def move_command(data):
    global commands
    commandPublisher.publish(commands)

def display_points(data):
    global point
    rvizPublisher.publish(point_2D)

if __name__ == '__main__':
    try:
        # Initialize ROS::node
        rospy.init_node('move', anonymous=True)
        rospy.Subscriber("/scan" , LaserScan , laserCallback)
        commands.linear.x = FORWARD_SPEED_MPS
        rospy.Timer(rospy.Duration(0.1), move_command, oneshot = False)
        rospy.Timer(rospy.Duration(0.1), display_points, oneshot = False)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass