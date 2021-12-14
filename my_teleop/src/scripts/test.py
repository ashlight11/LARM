#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('MOVER TWIST', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        value = Twist()
        value.linear.x = 2.0
        value.angular.z = -1.8
        rospy.loginfo(value)
        pub.publish(value)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
