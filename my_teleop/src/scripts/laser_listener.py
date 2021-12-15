#!/usr/bin/python3
import math, rospy
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

def callback(data):
    laserData = data

    mean_utmost_left = 0.0
    mean_front_left = 0.0
    mean_front = 0.0
    mean_front_right = 0.0
    mean_utmost_right = 0.0

    nb_values = len(laserData.ranges)
    for index, value in enumerate(laserData.ranges) : 
        if (index < 180 and value < 0.45) :
            mean_front_left = value
        elif (index in range (180, 900) and value < 0.35) :
            mean_front = value
        elif(index > 900 and value < 0.45) :
            mean_front_right = value


    """ for i in range (nb_values - 1):
        if (i < math.floor(nb_values/5)) :
            mean_utmost_left += laserData.ranges[i]
        elif ( i in range (math.floor(nb_values/5), math.floor(2 * nb_values/5))) : 
            mean_front_left += laserData.ranges[i]
        elif ( i in range (math.floor(2 * nb_values/5), math.floor(3 * nb_values/5))) : 
            mean_front += laserData.ranges[i]
        elif ( i in range (math.floor(3 * nb_values/5), math.floor(4 * nb_values/5))) : 
            mean_front_right += laserData.ranges[i]
        else : 
            mean_utmost_right += laserData.ranges[i]
 """
            



    #mean_front = mean_front / 720
    #mean_front_left = mean_front_left/ 180
    #mean_front_right = mean_front_right / 180
    #mean_utmost_left = mean_utmost_left / (nb_values/5)
    #mean_utmost_right = mean_utmost_right / (nb_values/5)

    
    #print("range at utmost left : ", mean_utmost_left)
    print("range at right ", mean_front_left)
    print("range in front", mean_front)
    print("range at left ", mean_front_right)
    #print("range at utmost right : ", mean_utmost_right)

    #for i in range (len(laserData.ranges) - 1):
    #    rospy.loginfo("Range %d : value %f", i, laserData.ranges[i])


	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.angle_min)

# Initialize ROS::node
rospy.init_node('Listener', anonymous=True)
rospy.Subscriber("/base_scan" , LaserScan , callback)

commandPublisher = rospy.Publisher(
    '/cmd_vel_mux/input/navi',
    Twist, queue_size=10
)


# spin() enter the program in a infinite loop
print("Start laser_listener.py")
rospy.spin()


