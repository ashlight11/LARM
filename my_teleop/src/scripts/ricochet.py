#!/usr/bin/python3
import math, rospy, numpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 
from sensor_msgs.msg import PointCloud2

laserData = LaserScan()
commands = Twist()
FORWARD_SPEED_MPS = 0.7
TURN_SPEED_MPS = 0.2
ANGULAR_TURN = 2

def callback(data):
    laserData = data
    nb_values = len(laserData.ranges)
    obstacles= []
    
    angle= data.angle_min
    for aDistance in data.ranges :
        if 0.1 < aDistance and aDistance < 5.0 :
            aPoint= [ 
                math.cos(angle) * aDistance, 
                math.sin( angle ) * aDistance
            ]
            obstacles.append( aPoint )
        angle+= data.angle_increment
    rospy.loginfo( str(
        [[ round(p[0], 2), round(p[1], 2) ] for p in  obstacles[0:10] ] 
    ) + " ..." )
    for point in obstacles:
        value = PointCloud2()
        value.data = point
        rvizPublisher.publish(value)


rvizPublisher = rospy.Publisher(
    '/RVIZ_DATA',
    PointCloud2, queue_size=10
)


commandPublisher = rospy.Publisher(
    '/cmd_vel',
    Twist, queue_size=10
)

# Publish velocity commandes:
def move_robot():
    # Compute cmd_vel here and publish... (do not forget to reduce timer duration)
    commands.linear.x = FORWARD_SPEED_MPS
    commands.angular.z = 0
    commandPublisher.publish(commands)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        commandPublisher.publish(commands)
        rate.sleep()

# call the move_command at a regular frequency:
# rospy.Timer( rospy.Duration(0.1), move_command, oneshot=False )

# spin() enter the program in a infinite loop
print("Start move.py")

if __name__ == '__main__':
    try:
        # Initialize ROS::node
        rospy.init_node('move', anonymous=True)
        rospy.Subscriber("/scan" , LaserScan , callback)

        move_robot()
        # spin() enter the program in a infinite loop
        print("Start ricochet.py")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass