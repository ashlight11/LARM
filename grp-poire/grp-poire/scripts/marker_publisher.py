from __future__ import print_function
import numpy
import rospy
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped

class MarkerPublisher():
    def __init__(self):
        self.tfListener = tf.TransformListener()
        rospy.Subscriber("alert_bottle", Pose , self.handle_markers)
        # publisher for bottle markers
        self.markerPublisher = rospy.Publisher(
            '/bottle',
            Marker, queue_size=10
        )
        self.markers_list = list()


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
            #print("Marker Published because first one")

        for existing_marker in self.markers_list:
            if self.areNotInSameArea(existing_marker.pose, pose):
                marker_to_publish = self.generateMarker(pose)
                self.markerPublisher.publish(marker_to_publish)
                self.markers_list.append(marker_to_publish)
                #print("Marker Published because not in area")

            elif self.positionChangedSignificantly(existing_marker.pose, pose):
                # Delete current marker 
                existing_marker.action = Marker.DELETE
                self.markerPublisher.publish(existing_marker)
                self.markers_list.remove(existing_marker)
                #print("Marker Deleted because position changed")
                newest_marker = self.generateMarker(pose)
                self.markerPublisher.publish(newest_marker)
                self.markers_list.append(newest_marker)
                #print("Marker Published because position changed")

    def areNotInSameArea(self, existing_marker: Pose, new_marker: Pose):
        a = numpy.array((existing_marker.position.x, existing_marker.position.y))
        b = numpy.array((new_marker.position.x, new_marker.position.y))
        dist = numpy.linalg.norm(a-b)
        #print("!!!! DISTANCE : " + str(dist) + " are in same area : " + str(dist<10))
        return dist > 5

    def positionChangedSignificantly(self, existing_marker: Pose, new_marker: Pose):
        diff_x = abs(existing_marker.position.x - new_marker.position.x)
        diff_y = abs(existing_marker.position.y - new_marker.position.y)
        #print("delta x : " + str(diff_x) + "; delta y : " + str(diff_y))
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
        print("MARKER ID : " + str(marker.id))
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

if __name__ == '__main__':
    try:
        # Initialize ROS::node
        rospy.init_node('Marker Publisher', anonymous=True)
        node = MarkerPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
