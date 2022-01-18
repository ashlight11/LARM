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

        self.isTurning = False

        self.mode = rospy.get_param(MODE_PARAMETER)

        # publisher for nav commands
        self.commandPublisher = rospy.Publisher(
            cmd_topic,
            Twist, queue_size=10)

        if(self.mode):
            #print("In simulation mode")
            self.FORWARD_SPEED_MPS = 2
        else:
            self.FORWARD_SPEED_MPS = 0.2

        self.commands.linear.x = self.FORWARD_SPEED_MPS
        self.commands.angular.z = 0.0

        # publisher for 2D Points
        self.rvizPublisher = rospy.Publisher(
            rviz_topic,
            PointCloud, queue_size=10
        )

        if(self.mode):
            rospy.Subscriber(laser_topic, LaserScan, self.laser_callback_sim)
            rospy.Timer(rospy.Duration(PUBLISHING_RATE),
                        self.move_command, oneshot=False)
        else:
            rospy.Subscriber(laser_topic, LaserScan, self.laser_callback)

    def move_command(self, data):
        if(not self.isTurning):
            self.commandPublisher.publish(self.commands)

    def display_points(self, data):
        self.point_2D.header.frame_id = 'base_link'
        self.rvizPublisher.publish(self.point_2D)

    def laser_callback_sim(self, data: LaserScan):
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

        thr1 = 1  # Laser scan range threshold
        thr2 = 1  # Turning threshold

        # Checks if there are obstacles in front and
        if front > thr1 and left > thr2 and right > thr2 and not isTurning:
            # 15 degrees left and right 
            # go forward (linear velocity)
            self.commands.linear.x = self.FORWARD_SPEED_MPS
            self.commands.angular.z = 0.0  # do not rotate (angular velocity)
        else:
            
            '''if(front < thr1):
                self.commands.linear.x = 0.0
                if(left < right):
                    self.commands.angular.z = - data.angle_max
                    isTurning = True
                else:
                    self.commands.angular.z = data.angle_max'''
            if (left < thr2):

                self.commands.linear.x = 0.0  # stop
                self.commands.angular.z = data.angle_max  # rotate counter-clockwise
                isTurning = True
            elif (right < thr2):

                self.commands.linear.x = 0.0  # stop
                self.commands.angular.z = - data.angle_max  # rotate clockwise
                isTurning = True

            if front > thr1 and (left > thr2) and (right > thr2):
                isTurning = False
                self.commands.linear.x = self.FORWARD_SPEED_MPS
                self.commands.angular.z = 0.0

    def laser_callback(self, data: LaserScan):
        self.point_2D.points.clear()
        obstacles = []
        angle = data.angle_min
        angle_left = 0
        angle_right = 0
        if(self.mode):  # if in simulation mode
            thr1 = 1  # Laser scan range threshold
            thr2 = 1  # Turning threshold

        else:  # eventually adapt these parameters IRL
            thr1 = 0.3  # Laser scan range threshold
            thr2 = 0.25  # Turning threshold

        for index, aDistance in enumerate(data.ranges):
            if 0.1 < aDistance and aDistance < 2.0:
                aPoint = Point32()
                aPoint.x = math.cos(angle) * aDistance
                aPoint.y = math.sin(angle) * aDistance
                aPoint.z = 0.0
                obstacles.append(aPoint)
            angle += data.angle_increment

            if(aDistance < thr1 and aDistance >= data.range_min):
                if index in range(200, math.floor(len(data.ranges)/2)):
                    if ((index * 0.36 - 200 * 0.36) * math.pi / 180) > angle_right:
                        angle_right = (index * 0.36 - 200 *
                                       0.36) * math.pi / 180
                    self.isTurning = True
                elif index in range(math.floor(len(data.ranges)/2), 580):
                    self.isTurning = True
                    if (abs((index * 0.36 - 580 * 0.36) * math.pi / 180)) > abs(angle_left):
                        angle_left = (index * 0.36 - 580 *
                                      0.36) * math.pi / 180
                else:
                    #self.isTurning = False
                    self.commands.linear.x = self.FORWARD_SPEED_MPS
                    self.commands.angular.z = 0.0
                    # self.commandPublisher.publish(self.commands)
            else:
                #print("GO FORWARD")
                self.isTurning = False
                self.commands.linear.x = self.FORWARD_SPEED_MPS
                self.commands.angular.z = 0.0
        #print("ANGLE LEFT " + str(angle_left) + " ANGLE RIGHT : " + str(angle_right))
        if abs(angle_left) > abs(angle_right):
            self.isTurning = True
            self.commands.angular.z = angle_left
            self.commands.linear.x = 0.0
            # self.commandPublisher.publish(self.commands)
        elif angle_right != 0.0:
            self.isTurning = True
            self.commands.angular.z = angle_right
            self.commands.linear.x = 0.0
            # self.commandPublisher.publish(self.commands)
        self.commandPublisher.publish(self.commands)

        for point in obstacles:
            self.point_2D.points.append(point)


if __name__ == '__main__':
    rospy.init_node('Move and Avoid Obstacles', anonymous=True)
    node = AutonomousNav()
    rospy.spin()
