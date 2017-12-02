#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from cv_bridge import CvBridge, CvBridgeError


STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.int = 0

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)
        
    def pose_cb(self, msg):
        self.pose = msg
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
   
    
    def traffic_cb(self, msg):
        self.lights = msg
        
    def getState():
        closestDist = sys.maxsize
        state = Null
        for light in self.lights:
            x = light.pose.position.x - self.current_x
            y = light.pose.position.y - self.current_y
            z = light.pose.position.z - self.current_z
            dist_sq = x*x + y*y + z*z
            if dist_sq < closestDist:
                closestDist = dist_sq
                state = light.state
        return state
   
    def image_cb(self, msg):
        image = bridge.imgmsg_to_cv2(cv_image, desired_encoding="bgr8")
        state = getState()
        if state:
            title = str(state) + str(self.int)
            int += 1
            imwrite(~/cc-cnd-fork/data/title, image)
        
        



if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
