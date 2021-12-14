#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
def move() : 
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        move_cmd = Twist()
        move_cmd.linear.x += 1.0
        move_cmd.angular.z += 1.0
        pub.publish(move_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        pass


