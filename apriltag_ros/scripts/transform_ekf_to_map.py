#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf.transformations as tf

# Variables to store initial pose
initial_pose = None

def tag_robot_pose_callback(msg):
    global initial_pose
    if initial_pose is None:
        initial_pose = msg.pose.pose
        rospy.loginfo("Initial pose set from /tag_robot_poses.")

def odometry_callback(msg):
    global initial_pose
    if initial_pose is None:
        rospy.logwarn("Initial pose not yet received!")
        return

    # Transform odometry to align with the map frame
    transformed_odometry = Odometry()
    transformed_odometry.header = msg.header
    transformed_odometry.child_frame_id = msg.child_frame_id

    # Extract position and orientation
    odom_pos = msg.pose.pose.position
    odom_ori = msg.pose.pose.orientation

    # Initial pose
    init_pos = initial_pose.position
    init_ori = initial_pose.orientation

    # Apply position transformation
    transformed_odometry.pose.pose.position.x = odom_pos.x + init_pos.x
    transformed_odometry.pose.pose.position.y = odom_pos.y + init_pos.y
    transformed_odometry.pose.pose.position.z = odom_pos.z + init_pos.z

    # Apply orientation transformation (quaternion multiplication)
    init_quat = [init_ori.x, init_ori.y, init_ori.z, init_ori.w]
    odom_quat = [odom_ori.x, odom_ori.y, odom_ori.z, odom_ori.w]
    result_quat = tf.quaternion_multiply(init_quat, odom_quat)

    transformed_odometry.pose.pose.orientation.x = result_quat[0]
    transformed_odometry.pose.pose.orientation.y = result_quat[1]
    transformed_odometry.pose.pose.orientation.z = result_quat[2]
    transformed_odometry.pose.pose.orientation.w = result_quat[3]

    # Publish transformed odometry
    pub.publish(transformed_odometry)

if __name__ == "__main__":
    rospy.init_node("transform_odometry_node")

    # Subscribers
    rospy.Subscriber("/tag_robot_poses", PoseWithCovarianceStamped, tag_robot_pose_callback)
    rospy.Subscriber("/odometry/filtered_global", Odometry, odometry_callback)

    # Publisher
    pub = rospy.Publisher("/ekf_odometry_map", Odometry, queue_size=10)

    rospy.spin()