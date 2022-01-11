from __future__ import print_function
import cv2 as cv
import argparse
import numpy, math
import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class BottleDetection():
    def __init__(self) :
        nuka_cascade_name = "/home/marianne.de.poorter/catkin_ws/src/LARM/grp-poire/grp-poire/src/scripts/cascade.xml"
        self.nuka_cascade = cv.CascadeClassifier()

        #-- 1. Load the cascades
        if not self.nuka_cascade.load(cv.samples.findFile(nuka_cascade_name)):
            print('--(!)Error loading face cascade')
            exit(0)
        #camera_device = args.camera
        #-- 2. Read the video stream
        '''self.cap = cv.VideoCapture(camera_device)
        if not self.cap.isOpened:
            print('--(!)Error opening video capture')
            exit(0)'''
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depthCallback)
        rospy.Subscriber("camera/color/image_raw", Image, self.rgbImageCallback)

        # publisher for bottle markers
        self.markerPublisher = rospy.Publisher(
            '/bottle',
            Marker, queue_size=10
        )
    
    def detectAndDisplay(self, frame):
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        frame_gray = cv.equalizeHist(frame_gray)
        color_info = (255, 255, 255)

        #-- Detect bottles
        bottles = self.nuka_cascade.detectMultiScale(frame_gray, minNeighbors = 30, scaleFactor = 3)
        for (x,y,w,h) in bottles:
            crop_frame = frame[y:y+h, x:x+w]
            median = numpy.median(crop_frame)
            #print("height " + str(frame.shape[0])  + "  width : " + str(frame.shape[1]))
            if median > 170 and y > frame.shape[0] / 3: 
                cv.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                print("bottle found")
                # process the detections
                #print(str(x) +" " + str(y) + " " +str(w) + " " +str(h))
                distance = 2000
                for row in self.depth_array[y:y+h, x:x+w] :
                    for pixel in row :
                        if pixel < distance :
                            distance = pixelros distance with realsense camera
                cv.waitKey(800)
                #cv.destroyAllWindows()
        #cv.imshow('Capture - Cola detection', frame)

    def depthCallback(self, data):
        bridge = CvBridge()
    
        # Try to convert the ROS Image message to a CV2 Image
        try:
            cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="16SC1")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        #show_image(cv_image)
        self.depth_array = numpy.array(cv_image, dtype=numpy.float32)
        

    def rgbImageCallback(self, data: Image):
        bridge = CvBridge()
        # Try to convert the ROS Image message to a CV2 Image
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        self.detectAndDisplay(cv_image)

    def show_image(self, img):
        '''cv.imshow("Image Window", img)
        cv.waitKey()
        cv.destroyAllWindows()'''
        print("show image")
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = 0
        marker.type = Marker.CUBE;
        marker.action = Marker.ADD;
        marker.pose.position.x = 1;
        marker.pose.position.y = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; 
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        self.markerPublisher.publish(marker)
        
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
        #node.show_video()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
