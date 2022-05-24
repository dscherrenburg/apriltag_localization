import rospy
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Pose, PoseWithCovariance
from tf.transformations import quaternion_from_matrix
import numpy as np


def from_matrix_to_transform_stamped(matrix, frame_id, child_frame_id):
    """creates a transform stamped message from a matrix"""
    new_tf = TransformStamped()
    new_tf.header.stamp = rospy.Time.now()
    new_tf.header.frame_id = frame_id
    new_tf.child_frame_id = child_frame_id
    
    new_tf.transform.translation.x = matrix[0][3]
    new_tf.transform.translation.y = matrix[1][3]
    new_tf.transform.translation.z = matrix[2][3]
    
    matrix_copy = matrix.copy()
    
    matrix_copy[0][3] = 0
    matrix_copy[1][3] = 0
    matrix_copy[2][3] = 0
    
    quat = quaternion_from_matrix(matrix_copy)
    
    new_tf.transform.rotation.w = quat[3]
    new_tf.transform.rotation.x = quat[0]
    new_tf.transform.rotation.y = quat[1]
    new_tf.transform.rotation.z = quat[2]
    
    return new_tf

def from_matrix_to_pose(matrix):
    pose = Pose()
    
    pose.position.x = matrix[0][3]
    pose.position.y = matrix[1][3]
    pose.position.z = matrix[2][3]
    
    matrix_copy = matrix.copy()
    matrix_copy[0][3] = 0
    matrix_copy[1][3] = 0
    matrix_copy[2][3] = 0
    
    quat = quaternion_from_matrix(matrix_copy)
    
    pose.orientation.w = quat[3]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    rospy.loginfo("pose: " + str(pose))
    return pose


def from_matrix_to_pose_cov_stamped(matrix):
    pose_with_cov_stamped = PoseWithCovarianceStamped()
    pose_with_cov_stamped.header.stamp = rospy.Time.now()
    pose_with_cov_stamped.header.frame_id = "map"
    
    pose_with_cov = PoseWithCovariance()
    pose_with_cov.pose = from_matrix_to_pose(matrix)
    pose_with_cov.covariance = np.reshape(np.identity(6), -1).tolist()
    
    pose_with_cov_stamped.pose = pose_with_cov
    return pose_with_cov_stamped

def from_matrix_to_tuple(matrix):
    x = matrix[0][3]
    y = matrix[1][3]
    z = matrix[2][3]
    
    matrix_copy = matrix.copy()
    matrix_copy[0][3] = 0
    matrix_copy[1][3] = 0
    matrix_copy[2][3] = 0
    
    quat = quaternion_from_matrix(matrix_copy)
    
    rw = quat[3]
    rx = quat[0]
    ry = quat[1] 
    rz = quat[2]
    return ((x, y, z), (rw, rx, ry, rz))

def distance_robot_tag(tf_robot_tag):
    translation = tf_robot_tag[0]
    squared_sum = 0
    for param in translation:
        squared_sum += param**2
    distance = np.sqrt(squared_sum)
    return distance

def distance_weights(distances):
    if len(distances) == 0:
        return None
    elif len(distances) == 1:
        return [1]
    
    weights = []
    total_distance = sum(distances)
    factors_sum = 0
    for distance in distances:
        factor = 1 - (distance / total_distance)
        factors_sum += factor
    
    for distance in distances:
        factor = 1 - (distance / total_distance)
        weight = factor / factors_sum
        weights.append(weight)
    
    return weights
    