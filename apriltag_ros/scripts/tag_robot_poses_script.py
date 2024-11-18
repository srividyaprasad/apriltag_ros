#!/usr/bin/env python

import rospy
import tf2_ros
import yaml
import os
import numpy as np
from tf.transformations import quaternion_about_axis, quaternion_inverse, quaternion_matrix, quaternion_from_euler, quaternion_multiply
from apriltag_ros.msg import AprilTagDetectionArray
import tf_conversions
import geometry_msgs.msg
import math

# Global variable to store the static transform
tf_buffer = None
static_tf_pub = None
static_transform_base_cam = None
pose_history = []

def get_static_transform():
    global static_transform_base_cam, tf_buffer
    try:
        # Fetch the static transform once at startup
        static_transform_base_cam = tf_buffer.lookup_transform('base_link', frame, rospy.Time(0), rospy.Duration(5.0))
        rospy.loginfo("Static transform successfully retrieved.")
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Failed to retrieve static transform: %s", e)

def load_tag_poses(yaml_file):
    """Load tag poses from a YAML file and store inverted transformations."""
    with open(yaml_file, 'r') as file:
        config = yaml.safe_load(file)
    
    tags = {}
    for body in config['bodies']:
        for body_name, body_data in body.items():
            if 'tags' in body_data:
                for tag in body_data['tags']:
                    tag_id = tag['id']
                    position = np.array([
                        tag['pose']['position']['x'],
                        tag['pose']['position']['y'],
                        tag['pose']['position']['z']
                    ])
                    
                    axis_angle = np.array([
                        tag['pose']['rotation']['x'],
                        tag['pose']['rotation']['y'],
                        tag['pose']['rotation']['z']
                    ])
                    angle = np.linalg.norm(axis_angle)
                    
                    axis = axis_angle / angle if angle > 0 else [1, 0, 0]
                    quaternion = quaternion_about_axis(angle, axis)
                    inverted_quaternion = quaternion_inverse(quaternion)
                    inverted_position = -np.dot(quaternion_matrix(inverted_quaternion)[:3, :3], position)
                    
                    tags[tag_id] = {
                        'position': position.tolist(),
                        'quaternion': quaternion.tolist(),
                        'inverted_position': inverted_position.tolist(),
                        'inverted_quaternion': inverted_quaternion.tolist()
                    }
    
    return tags

def publish_static_tag_tfs(yaml_file):
    """Publish static tag transforms using tf2."""
    global static_tf_pub
    tags = load_tag_poses(yaml_file)
    transforms = []

    # # Define rotation of Frame B relative to Frame A (e.g., 45° about Z-axis)
    # rotation_quaternion = quaternion_from_euler(0, 0, 3.14159 / 2)  # 45 degrees in radians
    
    # # Define translation (if Frame B is shifted relative to Frame A)
    # translation = [0.0, 0.0, 0.0]  # Example: 1m along X

    # transform = geometry_msgs.msg.TransformStamped()
    # transform.header.stamp = rospy.Time.now()
    # transform.header.frame_id = "world"
    # transform.child_frame_id = "tag_world" 
    # transform.transform.translation.x = translation[0]
    # transform.transform.translation.y = translation[1]
    # transform.transform.translation.z = translation[2]
    
    # transform.transform.rotation.x = rotation_quaternion[0]
    # transform.transform.rotation.y = rotation_quaternion[1]
    # transform.transform.rotation.z = rotation_quaternion[2]
    # transform.transform.rotation.w = rotation_quaternion[3]    
    
    # transforms.append(transform)

    for tag_id, pose in tags.items():
        position, quaternion = pose['position'], pose['quaternion']

        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "world"
        transform.child_frame_id = f"tag_{tag_id}" 
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = position
        transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w = quaternion

        transforms.append(transform)

    static_tf_pub.sendTransform(transforms)
    rospy.loginfo("Static transforms published successfully.")

def mul(tf1, tf2):
    """Multiplies two transforms and returns the combined transform."""
    
    # Create Pose objects from TransformStamped messages
    tf1_pose = geometry_msgs.msg.Pose()
    tf1_pose.position.x = tf1.transform.translation.x
    tf1_pose.position.y = tf1.transform.translation.y
    tf1_pose.position.z = tf1.transform.translation.z
    tf1_pose.orientation = tf1.transform.rotation

    tf2_pose = geometry_msgs.msg.Pose()
    tf2_pose.position.x = tf2.transform.translation.x
    tf2_pose.position.y = tf2.transform.translation.y
    tf2_pose.position.z = tf2.transform.translation.z
    tf2_pose.orientation = tf2.transform.rotation

    # Convert Pose to KDL Frame for transformation
    tf1_transform = tf_conversions.fromMsg(tf1_pose)
    tf2_transform = tf_conversions.fromMsg(tf2_pose)

    # Multiply the transforms
    combined_transform = tf1_transform * tf2_transform

    # Convert the result back to a TransformStamped
    result = geometry_msgs.msg.TransformStamped()
    result.transform.translation.x = combined_transform.p[0]
    result.transform.translation.y = combined_transform.p[1]
    result.transform.translation.z = combined_transform.p[2]

    # Convert the rotation matrix to a quaternion
    quaternion = combined_transform.M.GetQuaternion()
    result.transform.rotation.x = quaternion[0]
    result.transform.rotation.y = quaternion[1]
    result.transform.rotation.z = quaternion[2]
    result.transform.rotation.w = quaternion[3]
    result.header = tf1.header  # You can choose an appropriate frame and timestamp
    return result


def invert_transform(transform):
    """
    Inverts a TransformStamped.

    :param transform: TransformStamped
    :return: Inverted TransformStamped
    """
    inverted_transform = geometry_msgs.msg.TransformStamped()
    inverted_transform.header = transform.header

    # Swap the frame IDs
    inverted_transform.header.frame_id = transform.child_frame_id
    inverted_transform.child_frame_id = transform.header.frame_id

    # Extract translation and rotation
    translation = transform.transform.translation
    rotation = transform.transform.rotation

    # Convert quaternion to numpy array
    quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
   
    # Invert rotation
    inverted_quaternion = quaternion_inverse(quaternion)

    # Invert translation
    translation_vector = [-translation.x, -translation.y, -translation.z]
    inverted_translation = quaternion_matrix(inverted_quaternion)[:3, :3].dot(translation_vector)

    # Fill inverted transform
    inverted_transform.transform.translation.x = inverted_translation[0]
    inverted_transform.transform.translation.y = inverted_translation[1]
    inverted_transform.transform.translation.z = inverted_translation[2]

    inverted_transform.transform.rotation.x = inverted_quaternion[0]
    inverted_transform.transform.rotation.y = inverted_quaternion[1]
    inverted_transform.transform.rotation.z = inverted_quaternion[2]
    inverted_transform.transform.rotation.w = inverted_quaternion[3]

    return inverted_transform

def detection_callback(msg):
    """Callback for Apriltag detection."""
    global pose_history

    for detection in msg.detections:
        tag_id = detection.id[0]  

        cam_tag = tf_buffer.lookup_transform(frame, f'tag{tag_id}' , rospy.Time(0), rospy.Duration(5.0))
        
        distance = math.sqrt(detection.pose.pose.pose.position.x**2 + detection.pose.pose.pose.position.z**2 + detection.pose.pose.pose.position.y**2 ) 
        
        # Extract quaternion (x, y, z, w)
        qx = detection.pose.pose.pose.orientation.x
        qy = detection.pose.pose.pose.orientation.y
        qz = detection.pose.pose.pose.orientation.z
        qw = detection.pose.pose.pose.orientation.w

        rotation_matrix = quaternion_matrix([qx, qy, qz, qw])

        # Extract roll (rotation about the X-axis)
        roll = math.atan2(rotation_matrix[2][1], rotation_matrix[2][2])

        # Convert roll from radians to degrees if needed
        roll_deg = math.degrees(roll)

        # Extract pitch (rotation about the Y-axis)
        pitch = math.atan2(rotation_matrix[2][0], math.sqrt(rotation_matrix[2][1]**2 + rotation_matrix[2][2]**2))

        # Convert pitch from radians to degrees if needed
        pitch_deg = math.degrees(pitch)

        # Print both roll and pitch
        rospy.loginfo(f"DISTANCE: {distance}")
        rospy.loginfo(f"ROLL: {180-abs(roll_deg)}")
        rospy.loginfo(f"PITCH: {abs(pitch_deg)}")


        if (distance<2.5): # or (abs(pitch_deg)<=2 and (180-abs(roll_deg))<=2)):
            base_tag = mul(static_transform_base_cam, cam_tag)
            world_tag = tf_buffer.lookup_transform('world', f'tag_{tag_id}', rospy.Time(0), rospy.Duration(5.0))

            tag_base = invert_transform(base_tag) #inverse of base_tag
            world_base = mul(world_tag, tag_base) 

            tag_robot_pose = geometry_msgs.msg.PoseWithCovarianceStamped()
            tag_robot_pose.header.stamp = msg.header.stamp
            tag_robot_pose.header.frame_id = 'world'
            tag_robot_pose.pose.pose.position = world_base.transform.translation
            tag_robot_pose.pose.pose.orientation = world_base.transform.rotation
            tag_robot_pose.pose.covariance = [0.0]*36
            pub.publish(tag_robot_pose)

            # Add to history
            pose_history.append(tag_robot_pose)

            # Publish pose history
            pose_history_msg = geometry_msgs.msg.PoseArray()
            pose_history_msg.header.frame_id = 'world'
            pose_history_msg.header.stamp = rospy.Time.now()
            for pose in pose_history:
                pose_stamped = geometry_msgs.msg.Pose()
                pose_stamped.position = pose.pose.pose.position
                pose_stamped.orientation = pose.pose.pose.orientation
                pose_history_msg.poses.append(pose_stamped)

        history_pub.publish(pose_history_msg)
    
if __name__ == '__main__':
    try:
        rospy.init_node('tag_pose_transformer')
        frame = rospy.get_param('~camera_frame', 'camera_color_optical_frame')
        yaml_path = rospy.get_param('~yaml_path')

        rospy.loginfo(f"Using frame: {frame}")
        rospy.loginfo(f"Using YAML path: {yaml_path}")
        pub = rospy.Publisher('/tag_robot_poses', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)

        history_pub = rospy.Publisher('/tag_pose_history', geometry_msgs.msg.PoseArray, queue_size=10)

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        static_tf_pub = tf2_ros.StaticTransformBroadcaster()
        
        publish_static_tag_tfs(yaml_path)
        while(static_transform_base_cam is None and not rospy.is_shutdown()):
            print(static_transform_base_cam)
            get_static_transform()

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, detection_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    except FileNotFoundError:
        rospy.logerr("YAML file not found. Please check the file path.")