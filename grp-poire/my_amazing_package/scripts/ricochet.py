#!/usr/bin/python3
import math, rospy, numpy,time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 

laserData = LaserScan()
commands = Twist()
FORWARD_SPEED_MPS = 0.25
TURN_SPEED_MPS = 0.1
ANGULAR_TURN = 2.5

def callback(data):
    laserData = data
    nb_values = len(laserData.ranges)
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
        if ( point[0] > 0.1 and point[0] < 0.5 and abs(point[1]) < 0.3):
            if(point[0] < 0.2):
                commands.linear.x = - FORWARD_SPEED_MPS
            
            if (numpy.amax(right) > numpy.amax(left)):
                rospy.Timer(rospy.Duration(0.1), move_right, oneshot = True)
                
                
            
            else :
                rospy.Timer(rospy.Duration(0.1), move_left, oneshot = True)

                
        else :
            commands.linear.x = FORWARD_SPEED_MPS
            commands.angular.z = 0.0



commandPublisher = rospy.Publisher(
    '/cmd_vel_mux/input/navi',
    Twist, queue_size=10
)


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


print("Start move.py")

if __name__ == '__main__':
    try:
        # Initialize ROS::node
        rospy.init_node('move', anonymous=True)
        rospy.Subscriber("scan" , LaserScan , callback)
        rospy.Timer(rospy.Duration(0.1), move_command, oneshot = False)
        print("Start move_1_meter.py")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass