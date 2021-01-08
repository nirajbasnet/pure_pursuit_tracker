#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from pure_pursuit_utils import *
import os

class PurePursuitAnalysis:

    def __init__(self):
        rospy.init_node('speedDaemons_purepursuit_analysis', anonymous=True)
        dirname = os.path.dirname(__file__)
        original_path_filename = os.path.join(dirname, '../waypoints/original_waypoints.csv')
        self.original_waypoints = read_waypoints_from_csv(original_path_filename)
        executed_path_filename = os.path.join(dirname, '../waypoints/executed_waypoints.csv')
        self.executed_waypoints = read_waypoints_from_csv(executed_path_filename)
        self.max_error = 0.0
        self.total_abs_error = 0.0
        self.error_distances=[]
        self.calculate_error()

    def calculate_error(self):
        for index in range(len(self.original_waypoints)):
            error = dist(self.original_waypoints[index],self.executed_waypoints[index])
            self.total_abs_error += error
            self.error_distances.append(error)
        self.max_error = max(self.error_distances)
        print "Max error = ",self.max_error
        print "Total absolute error = ",self.total_abs_error



if __name__ == '__main__':
    try:
        error_analysis = PurePursuitAnalysis()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()


