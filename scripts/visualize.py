#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Vector3, Point,PoseStamped
from std_msgs.msg import Float32, String, ColorRGBA, Header
import tf.transformations as transform
from race.msg import drive_param
from speed_daemons_pure_pursuit.msg import pure_pursuit_param
from pure_pursuit_utils import *
from nav_msgs.msg import Path, Odometry
import math


class Visualizer:
    def __init__(self):
        rospy.init_node('pure_pursuit_visualizer')
        filename = rospy.get_param('~waypoints_filepath', '')
        self.waypoints = read_waypoints_from_csv(filename)

        # We will publish Marker type messages to this topic. When opening Rviz, we select this topic for visualization (just as we select topic /scan, say) and see the markers
        self.publisher_actual_velocity = rospy.Publisher('/visualization_actual_velocity', Marker, queue_size=1)
        self.window_publisher = rospy.Publisher('/visualization_window', MarkerArray, queue_size=1)
        self.reference_path_pub = rospy.Publisher('/reference_path', Path, queue_size=1)
        rospy.Subscriber('drive_parameters', drive_param, self.actual_speed_callback)
        rospy.Subscriber('pure_pursuit_parameters', pure_pursuit_param, self.pure_pursuit_callback)
        self.TURN_WINDOW_MIN = rospy.get_param('turn_window_min', 3.5)
        self.TURN_WINDOW_MAX = rospy.get_param('turn_window_max', 4.25)
        self.LOOKAHEAD_DIST= rospy.get_param('initial_lookahead_distance', 1.5)
        self.actual_velocity = str(0.0)

    def create_header(self, frame_id):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        return header

    def actual_speed_callback(self, data):
        self.actual_velocity = "True speed:" + str(data.velocity) + " m/s"

    def pure_pursuit_callback(self,data):
        self.LOOKAHEAD_DIST = data.lookahead_dist

    def publish_text(self, text, x, y, publisher):
        marker = Marker()
        # Specify the frame in which to interpret the x,y,z coordinates. It is the laser frame.
        marker.header.frame_id = "/base_link"
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0  # or set this to 0
        marker.type = marker.TEXT_VIEW_FACING
        marker.text = text
        marker.lifetime = rospy.Duration(0.1)
        marker.scale = Vector3(0.2, 0.2, 0.2)
        marker.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)
        publisher.publish(marker)

    def publish_path(self, waypoints, publisher):
        # Visualize path derived from the given waypoints in the path
        path = Path()
        path.header = self.create_header('map')
        path.poses = []
        for point in waypoints:
            tempPose = PoseStamped()
            tempPose.header = path.header
            tempPose.pose.position.x = point[0]
            tempPose.pose.position.y = point[1]
            tempPose.pose.orientation.w = 1.0
            path.poses.append(tempPose)
        publisher.publish(path)

    def publish_lookahead_circles(self):
        markerArray = MarkerArray()
        markerArray.markers.append(self.get_cylinder_marker(ColorRGBA(1.0,0.0,0.0,0.4), self.TURN_WINDOW_MIN))
        markerArray.markers.append(self.get_cylinder_marker(ColorRGBA(0.0, 1.0, 0.0, 0.4), self.TURN_WINDOW_MAX))
        markerArray.markers.append(self.get_cylinder_marker(ColorRGBA(0.0, 0.0, 1.0, 0.4), self.LOOKAHEAD_DIST))
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        self.window_publisher.publish(markerArray)

    def get_cylinder_marker(self,color,radius):
        marker = Marker()
        marker.header.frame_id = "/laser"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale= Vector3(radius, radius, 0.06)
        marker.color = color
        marker.pose.position = Point(0.0,0.0,0.0)
        return marker


if __name__ == '__main__':
    visualizer = Visualizer()
    rate = rospy.Rate(10)
    count = 0
    while not rospy.is_shutdown():
        # visualizer.publish_text(visualizer.actual_velocity, -1, -2, visualizer.publisher_actual_velocity)
        visualizer.publish_lookahead_circles()
        if(count>30):
            visualizer.publish_path(visualizer.waypoints,visualizer.reference_path_pub)
            count = 0
        count +=1
        rate.sleep()
