from __future__ import print_function
import cv2 as cv
import argparse
import rospy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


'''def detectAndDisplay(frame):
    frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frame_gray = cv.equalizeHist(frame_gray)

    # -- Detect faces
    faces = face_cascade.detectMultiScale(frame_gray)
    for (x, y, w, h) in faces:
        cv.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

    cv.imshow('Capture - Face detection', frame)
'''

'''parser = argparse.ArgumentParser(
    description='Code for Cascade Classifier tutorial.')
parser.add_argument('--face_cascade', help='Path to face cascade.',
                    default='cascade.xml')

args = parser.parse_args()

face_cascade_name = args.face_cascade

face_cascade = cv.CascadeClassifier()

# -- 1. Load the cascades
if not face_cascade.load(cv.samples.findFile(face_cascade_name)):
    print('--(!)Error loading face cascade')
    exit(0)

camera_device = args.camera
# -- 2. Read the video stream
cap = cv.VideoCapture(camera_device)
if not cap.isOpened:
    print('--(!)Error opening video capture')
    exit(0)

while True:
    ret, frame = cap.read()
    if frame is None:
        print('--(!) No captured frame -- Break!')
        break

    detectAndDisplay(frame)

    if cv.waitKey(10) & 0xFF == ord('q'):
        break'''

# publisher for bottle markers
markerPublisher = rospy.Publisher(
    '/bottle',
    Marker, queue_size=10
)

# Define a function to show the image in an OpenCV Window


def show_image(img):
    '''cv.imshow("Image Window", img)
    cv.waitKey()
    cv.destroyAllWindows()'''
    print("show image")
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.id = 0
    marker.type = Marker.SPHERE;
    marker.action = Marker.ADD;
    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 1;
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
    markerPublisher.publish()


def depthCallback(data):
    bridge = CvBridge()
    
    # Try to convert the ROS Image message to a CV2 Image
    try:
         cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="16SC1")
    except CvBridgeError as e:
         rospy.logerr("CvBridge Error: {0}".format(e))
    #show_image(cv_image)



def rgbImageCallback(data: Image):

    bridge = CvBridge()
    
    # Try to convert the ROS Image message to a CV2 Image
    try:
         cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
         rospy.logerr("CvBridge Error: {0}".format(e))

    show_image(cv_image)


if __name__ == '__main__':
    try:
        # Initialize ROS::node
        rospy.init_node('Bottle_Detection', anonymous=True)
        rospy.Subscriber("camera/depth/image_rect_raw", Image, depthCallback)
        rospy.Subscriber("camera/color/image_raw", Image, rgbImageCallback)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
