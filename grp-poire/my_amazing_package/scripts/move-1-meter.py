#!/usr/bin/python3
import math, rospy, numpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 

laserData = LaserScan()
commands = Twist()
FORWARD_SPEED_MPS = 0.2
TURN_SPEED_MPS = 0.1
ANGULAR_TURN = 4

def callback(data):
    global commands
    global laserData
    laserData = data

    nb_values = len(laserData.ranges)
    right = laserData.ranges[:math.floor(nb_values/6)]
    front = laserData.ranges[math.floor(2 * nb_values/6):math.floor(5 * nb_values/6)]
    left = laserData.ranges[math.floor(5 * nb_values/6):]


    for value in front : 
        if(value < 0.6):
            if (numpy.amax(right) > numpy.amax(left)):
                commands.linear.x = TURN_SPEED_MPS 
                commands.angular.z = - ANGULAR_TURN
                print("turn right", commands.angular.z)
            
            else :
                commands.linear.x = TURN_SPEED_MPS 
                commands.angular.z = ANGULAR_TURN
                print("turn left", commands.angular.z)
        
                
        else :
            commands.linear.x = FORWARD_SPEED_MPS
            commands.angular.z = 0


commandPublisher = rospy.Publisher(
    '/cmd_vel_mux/input/navi',
    Twist, queue_size=10
)

# Publish velocity commandes:
def move_robot():
    global commands
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
#rospy.spin()

if __name__ == '__main__':
    try:
        # Initialize ROS::node
        rospy.init_node('move', anonymous=True)
        rospy.Subscriber("/scan" , LaserScan , callback)
        move_robot()
        # spin() enter the program in a infinite loop
        print("Start move_1_meter.py")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
