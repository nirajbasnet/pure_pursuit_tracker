#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path, Odometry

pub = rospy.Publisher("pf/viz/inferred_pose", PoseStamped, queue_size=1)

racecar_pose = Pose()


# Gets the racecar pose from gazebo/model_states topic. Since Gazebo has multiple
# models (racecar, ground plane) we have to index for the "racecar".
def robot_pose_update(data):
    global racecar_pose
    racecar_pose = data.pose.pose
    msg = PoseStamped()
    msg.pose = racecar_pose
    pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("remap_simulator_pose")
    # Set the update rate
    # Set subscribers
    rospy.Subscriber("/odom", Odometry, robot_pose_update)
    rospy.spin()
