from os import name
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Point32
from sensor_msgs.msg import LaserScan, PointCloud
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import tf
import math
import numpy


PUBLISHING_RATE = 0.1
MODE_PARAMETER = "/simulation"
goal_topic = '/move_base_simple/goal'
cmd_topic = '/cmd_vel_mux/input/navi'
rviz_topic = '/rviz_point_cloud'
laser_topic = '/scan'

class AutonomousNav():

    def __init__(self):
        self.tfListener = tf.TransformListener()
        self.commands = Twist()  
        self.point_2D = PointCloud()

        self.mode = rospy.get_param(MODE_PARAMETER)

        # publisher for nav commands
        self.commandPublisher = rospy.Publisher(
            cmd_topic,
            Twist, queue_size=10)

        if(self.mode):
            #print("In simulation mode")
            self.FORWARD_SPEED_MPS = 2
        else:
            self.FORWARD_SPEED_MPS = 0.25

        self.commands.linear.x = self.FORWARD_SPEED_MPS
        self.commands.angular.z = 0.0

        # publisher for 2D Points
        self.rvizPublisher = rospy.Publisher(
            rviz_topic,
            PointCloud, queue_size=10
        )

        rospy.Subscriber(laser_topic, LaserScan, self.laser_callback)

        rospy.Timer(rospy.Duration(PUBLISHING_RATE), self.move_command, oneshot=False)
        

    def move_command(self, data):
        self.commandPublisher.publish(self.commands)

    def display_points(self, data):
        self.point_2D.header.frame_id = 'base_link'
        self.rvizPublisher.publish(self.point_2D)


    def laser_callback(self, data : LaserScan):
        isTurning = False
        self.point_2D.points.clear()
        obstacles = []
        angle = data.angle_min
        for aDistance in data.ranges:
            if 0.1 < aDistance and aDistance < 2.0:
                aPoint = Point32()
                aPoint.x = math.cos(angle) * aDistance
                aPoint.y = math.sin(angle) * aDistance
                aPoint.z = 0.0
                obstacles.append(aPoint)
            angle += data.angle_increment     
        for point in obstacles:
            self.point_2D.points.append(point)  
        front = data.ranges[0]
        left = data.ranges[15]
        right = data.ranges[345]
        '''print('-------------------------------------------')
        print('Range data at 0 deg:   {}'.format(front))
        print('Range data at 15 deg:  {}'.format(left))
        print('Range data at 345 deg: {}'.format(right))
        print('-------------------------------------------')'''

        if(self.mode) : # if in simulation mode
            thr1 = 1  # Laser scan range threshold
            thr2 = 1  # Turning threshold
        else : # eventually adapt these parameters IRL
            thr1 = 1  # Laser scan range threshold
            thr2 = 1  # Turning threshold

        if front > thr1 and left > thr2 and right > thr2 and not isTurning  :  # Checks if there are obstacles in front and
            # 15 degrees left and right (Try changing the
            # the angle values as well as the thresholds)
            # go forward (linear velocity)
            
            self.commands.linear.x = self.FORWARD_SPEED_MPS
            self.commands.angular.z = 0.0  # do not rotate (angular velocity)
        else:
            if(front < thr1 / 2) : 
                self.commands.linear.x = self.FORWARD_SPEED_MPS / 2 
            self.commands.linear.x = 0.0  # stop
            if (left < thr2):
                self.commands.angular.z = data.angle_max  # rotate counter-clockwise
                isTurning = True 
            elif (right < thr2):
                self.commands.angular.z = - data.angle_max # rotate clockwise
                isTurning = True
           
            if front > thr1 and left > thr2 and right > thr2:
                isTurning = False
                self.commands.linear.x = self.FORWARD_SPEED_MPS
                self.commands.angular.z = 0.0


if __name__ == '__main__':
    rospy.init_node('Move and Avoid Obstacles', anonymous=True)
    node = AutonomousNav()
    rospy.spin()
