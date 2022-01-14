from __future__ import print_function
from os import posix_fadvise
import cv2 as cv
import argparse
import numpy
import math
import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError


class BottleDetection():
    def __init__(self):
        nuka_cascade_name = "/home/marianne.de.poorter/catkin_ws/src/LARM/grp-poire/grp-poire/src/scripts/cascade.xml"
        self.nuka_cascade = cv.CascadeClassifier()

        # -- 1. Load the cascades
        if not self.nuka_cascade.load(cv.samples.findFile(nuka_cascade_name)):
            print('--(!)Error loading face cascade')
            exit(0)
        #camera_device = args.camera
        # -- 2. Read the video stream
        '''self.cap = cv.VideoCapture(camera_device)
        if not self.cap.isOpened:
            print('--(!)Error opening video capture')
            exit(0)'''
        self.tfListener = tf.TransformListener()
        rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, self.depthCallback)
        rospy.Subscriber("camera/color/image_raw",
                         Image, self.rgbImageCallback)
        rospy.Subscriber("/odom", Odometry, self.odomCoordinates)

        # publisher for bottle markers
        self.markerPublisher = rospy.Publisher(
            '/bottle',
            Marker, queue_size=10
        )
        self.markers_list = list()

    def detectAndDisplay(self, frame):
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        frame_gray = cv.equalizeHist(frame_gray)
        color_info = (255, 255, 255)
        camera_width = 1920.0
        camera_height = 1080.0
        hfov = 64

        # -- Detect bottles
        bottles = self.nuka_cascade.detectMultiScale(
            frame_gray, minNeighbors=30, scaleFactor=3)
        for (x, y, w, h) in bottles:
            crop_frame = frame[y:y+h, x:x+w]
            median = numpy.median(crop_frame)
            #print("height " + str(frame.shape[0])  + "  width : " + str(frame.shape[1]))
            if median > 170 and y > frame.shape[0] / 3:
                cv.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                print("bottle found")
                # process the detections
                #print(str(x) +" " + str(y) + " " +str(w) + " " +str(h))
                distance = 2000
                for row in self.depth_array[y:y+h, x:x+w]:
                    for pixel in row:
                        if pixel < distance and pixel != 0:
                            distance = pixel  # ros distance with realsense camera
                # D = H / sin(a) -> sin(a) = H / D -> a = arcsin(H/D)
                #angle = math.asin(w/distance)
                horizontal_angle_from_middle = ((x+w - camera_width/2)/(camera_width/2))*(hfov/2) * math.pi / 180
                print("!!! ANGLE : " + str(horizontal_angle_from_middle))
                cv.putText(frame, "{:f}".format(
                    distance), (x, y), cv.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv.LINE_AA)
                cv.putText(frame, "{:f}".format(
                    horizontal_angle_from_middle ), (x+w, y+h), cv.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv.LINE_AA)
                cv.imshow('Capture - Cola detection', frame)
                #cv.waitKey(800)
        
                estimated_pose = Pose()
                estimated_pose.position.x = distance / 1000 * math.cos(horizontal_angle_from_middle)
                estimated_pose.position.y = distance / 1000 * math.sin(horizontal_angle_from_middle) # equals distance * sin(angle from middle of camera)
                self.handle_markers(estimated_pose)

    def odomCoordinates(self, data: Odometry):
        self.position = data.pose

    def depthCallback(self, data):
        bridge = CvBridge()

        # Try to convert the ROS Image message to a CV2 Image
        try:
            cv_image = bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        # show_image(cv_image)
        self.depth_array = numpy.array(cv_image, dtype=numpy.float32)
        #print("min : " + str(numpy.amin(self.depth_array)) + "; max : " + str(numpy.amax(self.depth_array)))

    def rgbImageCallback(self, data: Image):
        bridge = CvBridge()
        # Try to convert the ROS Image message to a CV2 Image
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        self.detectAndDisplay(cv_image)

    def handle_markers(self, pose: Pose):
        
        '''if marker is not in area of already existing marker
                then publish marker
            else if position estimated has changed significantly
                then delete marker in area and publish new one
            else 
                do nothing'''
        if len(self.markers_list) == 0:
            marker_to_publish = self.generateMarker(pose)
            self.markerPublisher.publish(marker_to_publish)
            self.markers_list.append(marker_to_publish)
            #print(self.markers_list)
            #print(len(self.markers_list))
            print("Marker Published because first one")

        for existing_marker in self.markers_list:
            if not self.areInSameArea(existing_marker.pose, pose):
                marker_to_publish = self.generateMarker(pose)
                self.markerPublisher.publish(marker_to_publish)
                self.markers_list.append(marker_to_publish)
                print("Marker Published because not in area")
            elif self.positionChangedSignificantly(existing_marker.pose, pose):
                existing_marker.action = Marker.DELETE
                self.markerPublisher.publish(existing_marker)
                self.markers_list.remove(existing_marker)
                print("Marker Deleted because position changed")
                newest_marker = self.generateMarker(pose)
                self.markerPublisher.publish(newest_marker)
                self.markers_list.append(newest_marker)
                print("Marker Published because position changed")

    def areInSameArea(self, existing_marker: Pose, new_marker: Pose):
        a = numpy.array((existing_marker.position.x, existing_marker.position.y))
        b = numpy.array((new_marker.position.x, new_marker.position.y))
        dist = numpy.linalg.norm(a-b)
        print("!!!! DISTANCE : " + str(dist) + " are in same area : " + str(dist<10))
        return dist < 10

    def positionChangedSignificantly(self, existing_marker: Pose, new_marker: Pose):
        diff_x = abs(existing_marker.position.x - new_marker.position.x)
        diff_y = abs(existing_marker.position.y - new_marker.position.y)
        print("delta x : " + str(diff_x) + "; delta y : " + str(diff_y))
        bool_val = abs(existing_marker.position.x - new_marker.position.x) > 0.2 or abs(existing_marker.position.y - new_marker.position.y) > 0.4
        #print("!!!! CHANGED : " + str(bool_val))
        return bool_val

    def generateMarker(self, pose: Pose):
        pose_stamped = PoseStamped ()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = "map"
        pose_stamped = self.tfListener.transformPose(
            "map", pose_stamped)
        marker = Marker()
        marker.header.frame_id = "camera_link"
        marker.id = len(self.markers_list)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        return marker

    def show_video(self):
        while True:
            ret, frame = self.cap.read()
            if frame is None:
                print('--(!) No captured frame -- Break!')
                break
            self.detectAndDisplay(frame)
            if cv.waitKey(10) & 0xFF == ord('q'):
                break


if __name__ == '__main__':
    try:
        # Initialize ROS::node
        rospy.init_node('Bottle_Detection', anonymous=True)
        node = BottleDetection()
        # node.show_video()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
