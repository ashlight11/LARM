#!/usr/bin/python3
import math, rospy, numpy,time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import PointCloud2

# DEFINE CONSTANTS
laserData = LaserScan()
commands = Twist()
point_2D = PointCloud2()

# Callback function for laser data
def laserCallback(data):
    laserData = data
    nb_values = len(laserData.ranges)
    # splitting the right and left sides of the robot
    right = laserData.ranges[:math.floor(nb_values/6)]
    left = laserData.ranges[math.floor(5 * nb_values/6):]
    obstacles= []
    angle= data.angle_min
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
        if (isInDangerousZone(point)):
            #if the point is really close, speed up and reverse
            if(point[0] < 0.2):
                if(mode == "SIMULATION") :
                    commands.linear.x = - 2 * TURN_SPEED_MPS
                else :
                    commands.linear.x = - FORWARD_SPEED_MPS
            
            #if there is more room on the right side, go right
            if (numpy.amax(right) > numpy.amax(left)):
                rospy.Timer(rospy.Duration(0.1), move_right, oneshot = True)
            # else, go left
            else :
                rospy.Timer(rospy.Duration(0.1), move_left, oneshot = True)

        # if not dangerous, stay as it is      
        else :
            commands.linear.x = FORWARD_SPEED_MPS
            commands.angular.z = 0.0

def isInDangerousZone(point):
    if (mode == "SIMULATION"):
        return point[0] > 0.05 and point[0] < 0.5 and abs(point[1]) < 0.3
    else :
        return point[0] > 0.1 and point[0] < 0.5 and abs(point[1]) < 0.3

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

def display_points(data):
    global point
    rvizPublisher.publish(point_2D)

# publish at rate 10 Hz
def move_command(data):
    cmd = Twist()
    cmd.linear.x = FORWARD_SPEED_MPS
    commandPublisher.publish(cmd)



def move_right(data):
    cmd = Twist()
    cmd.linear.x = FORWARD_SPEED_MPS
    cmd.angular.z = - ANGULAR_TURN
    commandPublisher.publish(cmd)

def move_left(data):
    cmd = Twist()
    cmd.linear.x = FORWARD_SPEED_MPS
    cmd.angular.z = ANGULAR_TURN
    commandPublisher.publish(cmd)


if __name__ == '__main__':
    try:
        # Initialize ROS::node
        rospy.init_node('Move_And_Scan', anonymous=True)
        rospy.Subscriber("scan" , LaserScan , laserCallback)
        mode = rospy.get_param('MODE')
        if(mode == "SIMULATION") :
            FORWARD_SPEED_MPS = 1.5
            TURN_SPEED_MPS = 1
            ANGULAR_TURN = 2
        else :
            FORWARD_SPEED_MPS = 0.25
            TURN_SPEED_MPS = 0.1
            ANGULAR_TURN = 2.5
        rospy.Timer(rospy.Duration(0.1), move_command, oneshot = False)
        rospy.Timer(rospy.Duration(0.1), display_points, oneshot = False)
        print("Start move.py")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
