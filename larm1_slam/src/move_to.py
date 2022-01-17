from os import name
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import tf
import math
import numpy


FORWARD_SPEED_MPS = 2
ANGULAR_TURN = 1.5
goal_topic = '/move_base_simple/goal'
isFollowingGoal = False

class MyNode:

    def __init__(self):
        self.tfListener = tf.TransformListener()
        self.commands = Twist()  
        self.point_2D = PointCloud()

        # publisher for nav commands
        self.commandPublisher = rospy.Publisher(
            '/cmd_vel_mux/input/navi',
            Twist, queue_size=10)
        self.commands.linear.x = FORWARD_SPEED_MPS
        self.commands.angular.z = 0.0

        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.Subscriber(goal_topic, PoseStamped, self.goal_callback)
        rospy.Subscriber("/odom", Odometry, self.odomCoordinates)
        rospy.Timer(rospy.Duration(0.1), self.move_command, oneshot=False)

    def odomCoordinates(self, data: Odometry):
        global isFollowingGoal
        self.current_position = data.pose
        '''inc_x = self.goal.pose.position.x - data.pose.pose.position.x
        inc_y = self.goal.pose.position.y - data.pose.pose.position.y

        angle_to_goal = math.atan2(inc_y, inc_x)
        rot_q = self.goal.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        isFollowingGoal = True
        while abs(angle_to_goal - theta) > 0.1:
            self.commands.linear.x = 0.0
            self.commands.angular.z = 0.4
            print(abs(angle_to_goal - theta) )
        isFollowingGoal = False

        self.commands.linear.x = FORWARD_SPEED_MPS 
        self.commands.angular.z = 0.0'''
        

    def goal_callback(self, data):
        global isFollowingGoal 
        self.goal = data  # PoseStamped goal in odom
        # rospy.Time(0) constructs a Time instance that has special significance in TF contexts
        # it will cause lookupTransform(..) and friends to return the latest available data
        # for a specific transform, instead of the data at a specific point in time.
        self.goal.header.stamp = rospy.Time(0)
        self.local_goal = self.tfListener.transformPose(
            "base_footprint", self.goal) # PoseStamped goal in base_footprint
        #print(self.local_goal)
        self.angle_to_goal = math.atan2(self.local_goal.pose.position.y, self.local_goal.pose.position.x)
        print("ANGLE : " + str(self.angle_to_goal))
        
        rospy.Timer(rospy.Duration(5), self.turnForGoal, oneshot = True)
        isFollowingGoal = False
        a = numpy.array((self.goal.pose.position.x, self.goal.pose.position.y))
        b = numpy.array((self.current_position.pose.position.x, self.current_position.pose.position.y))
        dist = numpy.linalg.norm(a-b)
        '''while(dist > 0.1):
            self.commands.linear.x = FORWARD_SPEED_MPS / 2
            self.commands.angular.z = 0
            self.commandPublisher.publish(self.commands)
        '''
    def move_command(self, data):
        self.commandPublisher.publish(self.commands)

    def turnForGoal(self, data):
        global isFollowingGoal
        isFollowingGoal = True
        self.commands.linear.x = 0.0
        self.commands.angular.z = self.angle_to_goal
        self.commandPublisher.publish(self.commands)


    def laser_callback(self, data : LaserScan):
        isTurning = False
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
        thr1 = 1  # Laser scan range threshold
        thr2 = 1  # Turning threshold

        if front > thr1 and left > thr2 and right > thr2 and not isTurning  :  # Checks if there are obstacles in front and
            # 15 degrees left and right (Try changing the
            # the angle values as well as the thresholds)
            # go forward (linear velocity)
            
            self.commands.linear.x = FORWARD_SPEED_MPS
            if not isFollowingGoal :
                self.commands.angular.z = 0.0  # do not rotate (angular velocity)
        else:
            if(front < thr1 / 2) : 
                self.commands.linear.x = FORWARD_SPEED_MPS / 2 
            self.commands.linear.x = 0.0  # stop
            if (left < thr2):
                self.commands.angular.z = data.angle_max  # rotate counter-clockwise
                isTurning = True 
            elif (right < thr2):
                self.commands.angular.z = - data.angle_max # rotate clockwise
                isTurning = True
           
            if front > thr1 and left > thr2 and right > thr2 and not isFollowingGoal:
                isTurning = False
                self.commands.linear.x = FORWARD_SPEED_MPS
                self.commands.angular.z = 0.0


if __name__ == '__main__':
    rospy.init_node('Move_Test', anonymous=True)
    node = MyNode()
    rospy.spin()
