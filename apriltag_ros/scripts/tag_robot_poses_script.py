#!/usr/bin/env python

import rospy
import tf2_ros
import yaml
import os
import numpy as np
from tf.transformations import quaternion_about_axis, quaternion_inverse, quaternion_matrix
import geometry_msgs.msg
from apriltag_ros import ApriltagArrayStamped

def load_tag_poses(yaml_file):
    """Load tag poses from a YAML file, convert rotations from axis-angle to quaternion,
       and store the inverted position and quaternion."""
    with open(yaml_file, 'r') as file:
        config = yaml.safe_load(file)
    
    tags = {}
    for body in config['bodies']:
        for body_name, body_data in body.items():
            if 'tags' in body_data:
                for tag in body_data['tags']:
                    tag_id = tag['id']
                    # Original position
                    position = np.array([
                        tag['pose']['position']['x'],
                        tag['pose']['position']['y'],
                        tag['pose']['position']['z']
                    ])
                    
                    # Original axis-angle rotation to quaternion
                    axis_angle = np.array([
                        tag['pose']['rotation']['x'],
                        tag['pose']['rotation']['y'],
                        tag['pose']['rotation']['z']
                    ])
                    angle = np.linalg.norm(axis_angle)
                    
                    if angle > 0:
                        axis = axis_angle / angle
                    else:
                        axis = [1, 0, 0]  # Default axis if no rotation
                    quaternion = quaternion_about_axis(angle, axis)
                    
                    # Invert quaternion
                    inverted_quaternion = quaternion_inverse(quaternion)
                    
                    # Invert position
                    inverted_position = -np.dot(quaternion_matrix(inverted_quaternion)[:3, :3], position)
                    
                    # Store the original and inverted transformations
                    tags[tag_id] = {
                        'position': position.tolist(),
                        'quaternion': quaternion.tolist(),
                        'inverted_position': inverted_position.tolist(),
                        'inverted_quaternion': inverted_quaternion.tolist()
                    }
    
    return tags

def publish_static_tag_tfs(yaml_file):
    """Publish static tag transforms using tf2."""
    static_tf_pub = tf2_ros.StaticTransformBroadcaster()

    tags = load_tag_poses(yaml_file)

    transforms = []
    for tag_id, pose in tags.items():
        position = pose['position']
        quaternion = pose['quaternion']
        
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"
        transform.child_frame_id = f"tag_{tag_id}" 

        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        transforms.append(transform)

    # Publish all transforms at once
    static_tf_pub.sendTransform(transforms)
    rospy.loginfo("Static transforms published successfully.")

def transform_tfs(tagpose_cam):
    """Transform the tag pose from camera frame to base_link frame."""
    
    # Create a PoseStamped message for the tag pose in the camera frame
    tagpose_cam_stamped = geometry_msgs.msg.PoseStamped()
    tagpose_cam_stamped.header.stamp = rospy.Time.now()
    tagpose_cam_stamped.header.frame_id = 'camera_color_optical_frame'
    tagpose_cam_stamped.pose = tagpose_cam

    # Use tf2 to perform the transformation from camera to base_link
    try:
        tagpose_base_link_stamped = tf2_ros.transform_pose('base_link', tagpose_cam_stamped)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Transform failed: %s", e)
        return None
    
    # Return the transformed pose in the base_link frame
    return tagpose_base_link_stamped.pose

def invert_tfs(pose):
    """Invert the translation and orientation (quaternion) of a given pose."""
    # Invert position (negate the translation)
    inverted_position = geometry_msgs.msg.Point()
    inverted_position.x = -pose.position.x
    inverted_position.y = -pose.position.y
    inverted_position.z = -pose.position.z

    # Invert orientation (quaternion inverse)
    inverted_orientation = quaternion_inverse([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    # Create a new Pose object with the inverted position and orientation
    inverted_pose = geometry_msgs.msg.Pose()
    inverted_pose.position = inverted_position
    inverted_pose.orientation.x = inverted_orientation[0]
    inverted_pose.orientation.y = inverted_orientation[1]
    inverted_pose.orientation.z = inverted_orientation[2]
    inverted_pose.orientation.w = inverted_orientation[3]

    return inverted_pose

def transform_tfs_world(tag_id, tag_base):
    """Transform the tag pose from camera frame to base_link frame."""
    
    # Create a PoseStamped message for the tag pose in the camera frame
    tag_base_stamped = geometry_msgs.msg.PoseStamped()
    tag_base_stamped.header.stamp = rospy.Time.now()
    tag_base_stamped.header.frame_id = f'tag{tag_id}'
    tag_base_stamped.pose = tag_base

    # Use tf2 to perform the transformation from camera to base_link
    try:
        world_base_stamped = tf2_ros.transform_pose('world', tag_base_stamped)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Transform failed: %s", e)
        return None
    
    # Return the transformed pose in the base_link frame
    return world_base_stamped.pose

def detection_callback(msg):
    """Callback for Apriltag detection."""
    tag_robot_poses = []  # List to store transformed tag poses

    # Process detections
    for detection in msg.detections:
        tag_id = detection.id[0]  
        cam_tag = detection.pose.pose  # tag in camera frame from continuous detection

        # Transform tag pose from camera frame to base_link frame
        base_tag = transform_tfs(cam_tag)

        tag_base = invert_tfs(base_tag)

        # publish tag_base if necessary
        world_base = transform_tfs_world(tag_id, tag_base)

        if world_base:
            tag_robot_poses.append(world_base)

    # Publish transformed poses
    if tag_robot_poses:
        pub.publish(tag_robot_poses)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('tag_pose_transformer')

        # Define the publisher for the transformed tag poses
        pub = rospy.Publisher('/tag_robot_poses', geometry_msgs.msg.Pose, queue_size=10)
        
        # Load the YAML file and publish static tag transforms
        yaml_path = "/home/srividyaprasad/tagslam_root/src/tagslam/example/tagslam.yaml"
        publish_static_tag_tfs(yaml_path)  # Publish static world to tag transforms
        
        # Subscribe to Apriltag detections
        rospy.Subscriber('/tag_detections', ApriltagArrayStamped, detection_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except FileNotFoundError:
        rospy.logerr("YAML file not found. Please check the file path.")
