#!/usr/bin/python3
import math, rospy, numpy,time
from geometry_msgs.msg import Twist, Point32
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import PointCloud
import std_msgs.msg 


# DEFINE CONSTANTS
laserData = LaserScan()
commands = Twist()
point_2D = PointCloud()
MODE_PARAMETER = "/mode"
MODE_SIMULATION = "simulation"

# Callback function for laser data
def laserCallback(data):
    global point_2D
    laserData = data
    nb_values = len(laserData.ranges)
    # splitting the right and left sides of the robot
    right = laserData.ranges[:math.floor(nb_values/6)]
    left = laserData.ranges[math.floor(5 * nb_values/6):]
    obstacles= []
    angle= data.angle_min
    for aDistance in data.ranges :
        if 0.1 < aDistance and aDistance < 2.0 :
            aPoint : Point32[float] = [ 
                math.cos(angle) * aDistance, 
                math.sin( angle ) * aDistance,
                0
            ]
            obstacles.append( aPoint )
        angle+= data.angle_increment
    #rospy.loginfo( str(
        #[[ round(p[0], 2), round(p[1], 2) ] for p in  obstacles[0:10] ] 
    #) + " ..." )
    for index, point in enumerate(obstacles) :
        point_2D.points.append(point)
        # if point is in a dangerous zone 
        if (isInDangerousZone(point)):
            #if the point is really close, speed up and reverse
            if(point[0] < 0.2):
                if(mode == MODE_SIMULATION) :
                    commands.linear.x = - 2 * TURN_SPEED_MPS
                else :
                    commands.linear.x = - FORWARD_SPEED_MPS
            
            #if there is more room on the right side, go right
            if (numpy.amax(right) > numpy.amax(left)):
                move_right()
            # else, go left
            else :
                move_left()

        # if not dangerous, stay as it is      
        else :
            commands.linear.x = FORWARD_SPEED_MPS
            commands.angular.z = 0.0

def isInDangerousZone(point):
    if (mode == MODE_SIMULATION):
        return point[0] > 0.05 and point[0] < 0.5 and abs(point[1]) < 0.3
    else :
        return point[0] > 0.1 and point[0] < 0.5 and abs(point[1]) < 0.3

# publisher for nav commands
commandPublisher = rospy.Publisher(
    '/cmd_vel_mux/input/navi',
    Twist, queue_size=10
)

# publisher for 2D Pointsroslau
rvizPublisher = rospy.Publisher(
    '/rviz_point_cloud',
    PointCloud, queue_size=10
)

def display_points(data):
    global point_2D
    header = std_msgs.msg.Header()
    #header.stamp = rospy.Time.now()
    header.frame_id = 'base_link'
    point_2D.header = header
    rvizPublisher.publish(point_2D)

# publish at rate 10 Hz
def move_command(data):
    cmd = Twist()
    cmd.linear.x = FORWARD_SPEED_MPS
    commandPublisher.publish(cmd)

def move_right():
    cmd = Twist()
    cmd.linear.x = FORWARD_SPEED_MPS
    cmd.angular.z = - ANGULAR_TURN
    commandPublisher.publish(cmd)

def move_left():
    cmd = Twist()
    cmd.linear.x = FORWARD_SPEED_MPS
    cmd.angular.z = ANGULAR_TURN
    commandPublisher.publish(cmd)


if __name__ == '__main__':
    try:
        # Initialize ROS::node
        rospy.init_node('Scan_and_Move', anonymous=True)
    
        mode = rospy.get_param(MODE_PARAMETER)
        print(mode)

        if(mode == MODE_SIMULATION) :
            print("In simulation mode")
            FORWARD_SPEED_MPS = 1.5
            TURN_SPEED_MPS = 1
            ANGULAR_TURN = 2
        else :
            FORWARD_SPEED_MPS = 0.25
            TURN_SPEED_MPS = 0.1
            ANGULAR_TURN = 2.5
        rospy.Subscriber("scan" , LaserScan , laserCallback)
        rospy.Timer(rospy.Duration(0.1), move_command, oneshot = False)
        rospy.Timer(rospy.Duration(0.1), display_points, oneshot = False)
        print("Start move.py")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
