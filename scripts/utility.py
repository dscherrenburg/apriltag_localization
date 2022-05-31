import rospy
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Pose, PoseWithCovariance
from tf.transformations import quaternion_from_matrix
import numpy as np
import numpy.matlib as npm


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
    
    

# Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
# The quaternions are arranged as (w,x,y,z), with w being the scalar
# The result will be the average quaternion of the input. Note that the signs
# of the output quaternion can be reversed, since q and -q describe the same orientation

def averageQuaternions(Q):
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4,4))

    for i in range(0,M):
        q = Q[i,:]
        # multiply q with its transposed version q' and add A
        A = np.outer(q,q) + A

    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:,0].A1)


# Average multiple quaternions with specific weights
# The weight vector w must be of the same length as the number of rows in the
# quaternion maxtrix Q
def weightedAverageQuaternions(Q, w):
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4,4))
    weightSum = 0

    for i in range(0,M):
        q = Q[i,:]
        A = w[i] * np.outer(q,q) + A
        weightSum += w[i]

    # scale
    A = (1.0/weightSum) * A

    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)

    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]

    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:,0].A1)