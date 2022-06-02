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

def calculate_median(buffer):
    """Returns the median from the buffer
    -   buffer: list of poses in following form ([x, y, z], [qx, qy, qz, w])"""
    n_params = len(buffer[0][0]) 
    lsts, median= [], []
    for i in range(n_params):
        lsts.append([])

    # Looping through points and adding x, y, z to separate lists
    for detection in buffer:
        for i in range(n_params):
            lsts[i].append(detection[0][i])
    
    # Looping through parameter lists, sorting them to determine the median
    for lst in lsts:
        lst.sort()
        median.append(lst[int(len(lst)/2)])
    return median

def calculate_mean(buffer):
    """Returns the mean from the buffer
    -   buffer: list of poses in following form ([x, y, z], [qx, qy, qz, w])"""
    n_params = len(buffer[0][0]) 
    lsts, mean= [], []
    for i in range(n_params):
        lsts.append([])

    # Looping through points and adding x, y, z to separate lists
    for detection in buffer:
        for i in range(n_params):
            lsts[i].append(detection[0][i])
    
    # Looping through parameter lists, sorting them to determine the mean
    for lst in lsts:
        mean.append(np.mean(lst))
    return mean

def close_to_median(buffer, detection, max_error=0.1):
    """Returns True if error between the median of the current buffer and the detection is smaller than the max_error"""
    if len(buffer) == 0:
        return True
    
    median = calculate_median(buffer)

    if all([abs(median[i]-detection[0][i])<max_error for i in range(len(median))]):
        return True
    else:
        return False

def close_to_mean(buffer, detection, max_error=0.1):
    """Returns True if error between the mean of the current buffer and the detection is smaller than the max_error"""
    if len(buffer) == 0:
        return True
    
    mean = calculate_median(buffer)

    if all([abs(mean[i]-detection[0][i])<max_error for i in range(len(mean))]):
        return True
    else:
        return False

def median_outlier_filter(points, max_error=0.1):
    """Deletes the outliers from the list of points.
    -   points: list of points (each point is a list of translations)
    -   max_error: the maximum distance from the median"""
    
    n_params = len(points[0]) 
    lsts, median, filtered_lsts, filtered_points = [], [], [], []
    for i in range(n_params):
        lsts.append([])
        filtered_lsts.append([])

    # Looping through points and adding x, y, z to separate lists
    for point in points:
        for i in range(n_params):
            lsts[i].append(point[i])
    
    # Looping through parameter lists, sorting them to determine the median
    for lst in lsts:
        lst.sort()
        median.append(lst[int(len(lst)/2)])

    # Looping through points to check if the error from the median is too big. If so it deletes the entire point.
    for point in points:
        if all([(abs(point[i] - median[i]) < max_error) for i in range(n_params)]):
            filtered_points.append(point)
            for i in range(n_params):
                filtered_lsts[i].append(point[i])
        else:
            continue
    
    # If there where no valid points it returns just the points without filtering
    if len(filtered_lsts[0]) == 0:
        filtered_lsts = lsts
    
    return filtered_points, filtered_lsts