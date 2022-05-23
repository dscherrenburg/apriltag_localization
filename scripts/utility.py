import rospy
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Pose, PoseWithCovariance
from tf.transformations import quaternion_from_matrix


def from_matrix_to_transform_stamped(matrix, frame_id, child_frame_id):
    """creates a transform stamped message from a matrix"""
    new_tf = TransformStamped()
    new_tf.header.stamp = rospy.Time.now()
    new_tf.header.frame_id = frame_id
    new_tf.child_frame_id = child_frame_id
    
    new_tf.transform.translation.x = matrix[0][3]
    new_tf.transform.translation.y = matrix[1][3]
    new_tf.transform.translation.z = matrix[2][3]
    
    matrix[0][3] = 0
    matrix[1][3] = 0
    matrix[2][3] = 0
    
    quat = quaternion_from_matrix(matrix)
    
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
    
    matrix[0][3] = 0
    matrix[1][3] = 0
    matrix[2][3] = 0
    
    quat = quaternion_from_matrix(matrix)
    
    pose.orientation.w = quat[3]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    return pose


def from_matrix_to_pose_cov_stamped(matrix):
    pose = PoseWithCovarianceStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    
    pose_with_cov = PoseWithCovariance()
    pose_with_cov.pose = from_matrix_to_pose(matrix)
    
    pose.pose = pose_with_cov
    return pose