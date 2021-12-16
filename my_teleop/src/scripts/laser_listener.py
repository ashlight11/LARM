#!/usr/bin/python3
import math, rospy, numpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan 

# attributes for object LaserScan

#float32 angle_min        # start angle of the scan [rad]
#float32 angle_max        # end angle of the scan [rad]
#float32 angle_increment  # angular distance between measurements [rad]
#float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
#float32 scan_time        # time between scans [seconds]

#float32 range_min        # minimum range value [m]
#float32 range_max        # maximum range value [m]

#float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
#float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.

laserData = LaserScan()
commands = Twist()
FORWARD_SPEED_MPS = 0.5
LEFT = "LEFT"
RIGHT = "RIGHT"


def callback(data):
    laserData = data

    mean_front_left = 0.0
    mean_front = 0.0
    mean_front_right = 0.0


    nb_values = len(laserData.ranges)
    right = laserData.ranges[:math.floor(nb_values/3)]
    front = laserData.ranges[math.floor(nb_values/3):math.floor(2 * nb_values/3)]
    left = laserData.ranges[math.floor(2 * nb_values/3):]

    
    for value in front : 
        if(value < 0.5):
            if (numpy.amax(right) > numpy.amax(left)):
                commands.linear.x = 0.2
                commands.angular.z = -2
                print("turn right")
                break;
            else :
                commands.linear.x = 0.2
                commands.angular.z = 2
                print("turn left")
                break;
        else :
            commands.linear.x = FORWARD_SPEED_MPS
            commands.angular.z = 0

    # for index, value in enumerate(laserData.ranges) : 
    #     if (index < math.floor(nb_values/3) and value < 0.45) :
    #         mean_front_left = value
    #     elif (index in range (math.floor(nb_values/3), math.floor(2 * nb_values/3)) and value < 0.35) :
    #         mean_front = value
    #     elif(index > math.floor(2 * nb_values/3) and value < 0.45) :
    #         mean_front_right = value

    #print("range at right ", mean_front_left)
    #print("range in front", mean_front)
    #print("range at left ", mean_front_right)


    


def move_robot(direction = None):
    commands.linear.x = FORWARD_SPEED_MPS
    commands.angular.z = 0
    commandPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        commandPublisher.publish(commands)
        rate.sleep()
    rospy.loginfo("test : ", commands.angular.z)

# Initialize ROS::node


if __name__ == '__main__':
    try:
        rospy.init_node('Listener', anonymous=True)
        rospy.Subscriber("/base_scan" , LaserScan , callback)
        move_robot()
        # spin() enter the program in a infinite loop
        print("Start laser_listener.py")
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass



