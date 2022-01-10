from os import name
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud
from tf.transformations import euler_from_quaternion
import tf
import math
import numpy


FORWARD_SPEED_MPS = 1
ANGULAR_TURN = 1.5
goal_topic = '/move_base_simple/goal'
isFollowingGoal = False
isTurning = False


class MyNode:

    def __init__(self):
        self.tfListener = tf.TransformListener()
        self.commands = Twist()
        self.goal = PoseStamped()  # goal in odom
        self.local_goal = PoseStamped()  # goal in base_footprint
        self.point_2D = PointCloud()

        # publisher for nav commands
        self.commandPublisher = rospy.Publisher(
            '/cmd_vel_mux/input/navi',
            Twist, queue_size=10)
        self.commands.linear.x = FORWARD_SPEED_MPS
        self.commands.angular.z = 0.0

        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.Subscriber(goal_topic, PoseStamped, self.goal_callback)
        rospy.Timer(rospy.Duration(0.1), self.move_command, oneshot=False)

    def goal_callback(self, data):
        global isFollowingGoal
        self.goal = data
        isFollowingGoal = True
        self.local_goal = self.tfListener.transformPose(
            "/base_footprint", self.goal)
        print(self.local_goal)
        


    def move_command(self, data):
        global isFollowingGoal
        if(isFollowingGoal) : 
            angle_to_goal = math.atan2(self.local_goal.pose.position.y, self.local_goal.pose.position.x)
            while(abs(angle_to_goal) > 0.1 and isFollowingGoal):
                self.commands.linear.x = 0.0
                self.commands.angular.z = angle_to_goal
            if abs(angle_to_goal) < 0.1 :
                self.commands.linear.x = FORWARD_SPEED_MPS
                self.commands.angular.z = 0.0
                # Find how you know you reached the goal
        self.commandPublisher.publish(self.commands)


    def laser_callback(self, data):
        global isTurning
        global isFollowingGoal
        self.point_2D.points.clear()
        obstacles = []
        angle = data.angle_min
        for aDistance in data.ranges:
            if 0.1 < aDistance and aDistance < 2.0:
                aPoint = [
                    math.cos(angle) * aDistance,
                    math.sin(angle) * aDistance
                ]
                obstacles.append(aPoint)
            angle += data.angle_increment
        front = data.ranges[0]
        left = data.ranges[15]
        right = data.ranges[345]
        '''print('-------------------------------------------')
        print('Range data at 0 deg:   {}'.format(front))
        print('Range data at 15 deg:  {}'.format(left))
        print('Range data at 345 deg: {}'.format(right))
        print('-------------------------------------------')'''
        thr1 = 0.8  # Laser scan range threshold
        thr2 = 1

        if front > thr1 and left > thr2 and right > thr2 and not isTurning :  # Checks if there are obstacles in front and
            # 15 degrees left and right (Try changing the
            # the angle values as well as the thresholds)
            # go forward (linear velocity)
            
            self.commands.linear.x = FORWARD_SPEED_MPS
            if not isFollowingGoal :
                self.commands.angular.z = 0.0  # do not rotate (angular velocity)
        else:
            self.commands.linear.x = 0.0  # stop
            if (left < thr2):
                self.commands.angular.z = ANGULAR_TURN  # rotate counter-clockwise
                isTurning = True 
            elif (right < thr2):
                self.commands.angular.z = - ANGULAR_TURN  # rotate clockwise
                isTurning = True
           
            if front > thr1 and left > thr2 and right > thr2 :
                isTurning = False
                self.commands.linear.x = FORWARD_SPEED_MPS
                self.commands.angular.z = 0.0


if __name__ == '__main__':
    rospy.init_node('Move_Test', anonymous=True)
    node = MyNode()
    rospy.spin()
