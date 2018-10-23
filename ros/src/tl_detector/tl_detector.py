#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import tf
import cv2
import numpy as np
import yaml

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        # Change this to rospy.DEBUG to see the debug messages in the code
        rospy.init_node('tl_detector', log_level=rospy.INFO)


        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.camera_image = None
        self.lights = []
        self.lights_tree = None
        self.lights_2d = None
        self.tl_classifier = TLClassifier()

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.logdebug("Finished initializing tldetector")

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoints_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights
        # rospy.logdebug("lights: {}".format(self.lights))
        if not self.lights_2d:
            self.lights_2d = [[light_3d.pose.pose.position.x, light_3d.pose.pose.position.y] for light_3d in self.lights]
            # rospy.logdebug("Lights_2d: {}".format(self.lights_2d))
            self.lights_tree = KDTree(self.lights_2d)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

        rospy.logdebug("About to process new image")
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose_2d):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        pos_x = pose_2d[0]
        pos_y = pose_2d[1]
        closest_idx = self.waypoints_tree.query((pos_x, pos_y), 1)[1]

        # Check if closest is ahead or behind vehicle
        rospy.logdebug("Closest idx: {}".format(closest_idx))
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([pos_x, pos_y])
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx


    def get_closest_light_idx(self, waypoint):
        pos_x = waypoint[0]
        pos_y = waypoint[1]

        light_idx = self.lights_tree.query((pos_x, pos_y), 1)[1]
        rospy.logdebug("Light_idx: {}".format(light_idx))

        closest_light_coord = self.lights_2d[light_idx]
        prev_light_coord = self.lights_2d[light_idx - 1]
        cl_vect = np.array(closest_light_coord)
        prev_vect = np.array(prev_light_coord)
        pos_vect = np.array([pos_x, pos_y])
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        rospy.logdebug("About to determine whether light is in front, dot product val: {}".format(val))
        if val > 0:
            light_idx = (light_idx + 1) % len(self.lights_2d)
        return light_idx


    def get_light_state(self):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        rospy.logdebug("About to process traffic light")
        light_idx = None

        if(self.pose):
            rospy.logdebug("Yes we have pose")
            car_position = self.get_closest_waypoint((self.pose.pose.position.x, self.pose.pose.position.y))
            rospy.logdebug("Yes we have car position")
            light_idx = self.get_closest_light_idx(self.waypoints_2d[car_position])

        rospy.logdebug("Car position idx: {}  -- closest light idx: {}".format(car_position, light_idx))
        light_2d = self.lights_2d[light_idx]
        waypt_idx = self.get_closest_waypoint(light_2d)

        if waypt_idx is not None:
            state = self.get_light_state()
            rospy.logdebug("Light state: {}".format(state))
            return waypt_idx, state
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
