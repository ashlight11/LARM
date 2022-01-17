from __future__ import print_function
from os import posix_fadvise
import cv2 as cv
import numpy
import math
import rospy, rospkg
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError


# get an instance of RosPack with the default search paths
def get_pkg_path():
    rospack = rospkg.RosPack()
    return rospack.get_path('grp-poire')

class BottleDetectionOnly():
    def __init__(self) :
        nuka_cascade_name = get_pkg_path() + "/scripts/cascade.xml"
        self.nuka_cascade = cv.CascadeClassifier()
        self.camera_width = 1920.0
        self.camera_height = 1080.0
        self.hfov = 64

        # -- 1. Load the cascades
        if not self.nuka_cascade.load(cv.samples.findFile(nuka_cascade_name)):
            print('--(!)Error loading face cascade')
            exit(0)
        
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depthCallback)
        rospy.Subscriber("camera/color/image_raw", Image, self.rgbImageCallback)
        self.posePublisher = rospy.Publisher("alert_bottle", Pose, queue_size=10)

    def detectAndDisplay(self, frame):
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        frame_gray = cv.equalizeHist(frame_gray)
        color_info = (255, 255, 255)

        # -- Detect black bottles
        bottles = self.nuka_cascade.detectMultiScale(
            frame_gray, minNeighbors=30, scaleFactor=3)
        for (x, y, w, h) in bottles:
            crop_frame = frame[y:y+h, x:x+w]
            median = numpy.median(crop_frame)
            #print("height " + str(frame.shape[0])  + "  width : " + str(frame.shape[1]))
            if median > 170 and y > frame.shape[0] / 3:
                cv.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                estimated_pose = self.estimatePose(x,y, w, h)
                self.posePublisher.publish(estimated_pose)
                #cv.imshow('Capture - Cola detection', frame)
                #cv.waitKey(400)
                
        # Detect orange bottles 
        color = 8
        # on peut baisser la saturation pour inclure plus de pixels
        lo = numpy.array([color - 2, 240, 240]) # teinte légèrement plus basse
        hi = numpy.array([color + 2, 255, 255])# teinte légèrement plus foncée
        image = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        mask = cv.inRange(image, lo, hi)

        elements = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]
        if len(elements) > 0:
            c = max(elements, key=cv.contourArea)
            x,y,w,h = cv.boundingRect(c)
            if w > 10 and y < 500:
                cv.putText(frame, "Objet !", (int(x)+10, int(y) - 10),
                        cv.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv.LINE_AA)
                estimated_pose = self.estimatePose(x,y, w, h)
                self.posePublisher.publish(estimated_pose)
                #cv.imshow('Capture - Cola detection', frame)
                #cv.waitKey(400)

    def estimatePose(self, x, y, w, h):
        # process the detections
        #print(str(x) +" " + str(y) + " " +str(w) + " " +str(h))
        distance = 2000
        for row in self.depth_array[y:y+h, x:x+w]:
            for pixel in row:
                if pixel < distance and pixel != 0:
                    distance = pixel  # ros distance with realsense camera
        angle = ((x+w - self.camera_width/2)/(self.camera_width/2))*(self.hfov/2) * math.pi / 180
        estimated_pose = Pose()
        estimated_pose.position.x = distance / 1000 * math.cos(angle) # equals distance * cos(angle from middle of camera)
        estimated_pose.position.y = distance / 1000 * math.sin(angle)  # equals distance * sin(angle from middle of camera)
        return estimated_pose

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

    '''def show_video(self):
        while True:
            ret, frame = self.cap.read()
            if frame is None:
                print('--(!) No captured frame -- Break!')
                break
            self.detectAndDisplay(frame)
            if cv.waitKey(10) & 0xFF == ord('q'):
                break'''

if __name__ == '__main__':
    try:
    # Initialize ROS::node
        rospy.init_node('Bottle_Detection_Image_Processing', anonymous=True)
        node = BottleDetectionOnly()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    
    