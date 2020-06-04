#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''
CRUISE_SPEED = 11.1111 # m/s
LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number
BUFFER_TRAFFIC_LIGHT_WPS = 10 # Number of waypoints that car needs to stop in front of a red light

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/tl_color', Int32, self.tl_color_cb) # 0:red, 1:yellow, 2:green

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.pose = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.traffic_light_wp_idx = -1
        self.traffic_light_color = -1

        self.loop()


    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoint_tree:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
    	self.base_waypoints = waypoints
    	# TODO: Implement
    	if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    def publish_waypoints(self):
        lane = Lane()
        closest_idx = self.get_closest_waypoint_idx()
        light_ahead_wps = self.traffic_light_wp_idx - closest_idx

        # Red light too close, emergency brake
        if self.traffic_light_wp_idx > 0 and light_ahead_wps < BUFFER_TRAFFIC_LIGHT_WPS and self.traffic_light_color == 0:
            lane.waypoints = self.base_waypoints.waypoints[closest_idx:self.traffic_light_wp_idx]
            for wp in lane.waypoints:
                self.set_waypoint_velocity(wp, 0.0)

        # Slow down due to a detected red or yellow light
        elif self.traffic_light_wp_idx > 0 and light_ahead_wps < LOOKAHEAD_WPS:
            lane.waypoints = self.base_waypoints.waypoints[closest_idx:self.traffic_light_wp_idx]
            lowered_speed = float(light_ahead_wps/float(LOOKAHEAD_WPS)*CRUISE_SPEED)
            for wp in lane.waypoints:
                self.set_waypoint_velocity(wp, lowered_speed)

        # Cruise
        else:
            farthest_idx = closest_idx + LOOKAHEAD_WPS
            lane.waypoints = self.base_waypoints.waypoints[closest_idx:farthest_idx]
            for wp in lane.waypoints:
                self.set_waypoint_velocity(wp, CRUISE_SPEED)

        self.final_waypoints_pub.publish(lane)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_light_wp_idx = msg.data

    def tl_color_cb(self, msg):
        self.traffic_light_color = int(msg.data)

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
