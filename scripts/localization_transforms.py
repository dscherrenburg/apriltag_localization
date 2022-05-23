#!/usr/bin/python
import tf.transformations as tft
from quaternion_avg import averageQuaternions
import numpy as np

class SimpleRobotPosition:
    def __init__(self, x, y, z, rx, ry, rz, rw, frame_id="map", child_frame_id="base_link"):
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.rw = rw
        self.position = (x, y, z)
        self.quaternion = (rx, ry, rz, rw)
        self.normalize_quaternion()
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        
        
    def __str__(self):
        string = "Robot position: " + str(self.position) + "\n" + \
                 "Robot orientation: " + str(self.quaternion) + "\n"
        return string
        
    def set_position(self, x, y, z):
        """Set the position of the robot"""
        self.x = x
        self.y = y
        self.z = z
        self.position = (x, y, z)
    
    def set_rotation(self, rx, ry, rz, rw):
        """Set the rotation of the robot"""
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.rw = rw
        self.quaternion = (rx, ry, rz, rw)
        self.normalize_quaternion()
    
    def set_all(self, x, y, z, rx, ry, rz, rw):
        """Set the position and rotation of the robot"""
        self.set_position(x, y, z)
        self.set_rotation(rx, ry, rz, rw)
    
    @staticmethod
    def zero():
        """Return a zero position"""
        return SimpleRobotPosition(0, 0, 0, 0, 0, 0, 1)
    
    @staticmethod
    def from_tuple_of_lists(tuple_of_lists):
        """Creates simple robot position from format (translation [x, y, z], rotation [x, y, z, w])"""
        return SimpleRobotPosition(tuple_of_lists[0][0], tuple_of_lists[0][1], tuple_of_lists[0][2],
                                   tuple_of_lists[1][0], tuple_of_lists[1][1], tuple_of_lists[1][2], tuple_of_lists[1][3])
    
    @staticmethod
    def from_list_of_tuple_of_lists(list):
        """Returns a simple robot position from a list position in format [(translation [x, y, z], rotation [x, y, z, w])]"""
        return [SimpleRobotPosition.from_tuple_of_lists(tuple_of_lists) for tuple_of_lists in list]
    
    @staticmethod
    def from_matrix(matrix):
        """Creates simple robot position from a 4x4 matrix"""
        robot_position = SimpleRobotPosition.zero()
        m = matrix.copy()
        robot_position.set_position(m[0][3], m[1][3], m[2][3])
        m[0][3] = 0
        m[1][3] = 0
        m[2][3] = 0
        quaternion = tft.quaternion_from_matrix(m)
        robot_position.set_rotation(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        return robot_position
    
    def normalize_quaternion(self):
        """Normalizes the quaternion"""
        self.quaternion = tft.unit_vector(self.quaternion)




class RobotPositionEstimator:
    def __init__(self):
        pass

    
    def estimate_position(self, robot_positions):
        """Estimates the position of the robot based on the given robot positions"""
        average_estimated_position = RobotPositionEstimator.average_of_robot_positions(robot_positions)
        return average_estimated_position
    
    @staticmethod
    def average_of_robot_positions(list):
        """Calculates the average of a list of robot positions"""
        x = sum([position.x for position in list]) / len(list)
        y = sum([position.y for position in list]) / len(list)
        z = sum([position.z for position in list]) / len(list)
        
        q = []
        for pos in list:
            q.append([pos.rw, pos.rx, pos.ry, pos.rz])
            
        q_avg = averageQuaternions(np.matrix(q))
        return SimpleRobotPosition(x, y, z, q_avg[1], q_avg[2], q_avg[3], q_avg[0])


def test():
    test_list = [([4.075312633720695, 1.2360087732444194, 1.202706453961071], [0.6612972918622178, -0.27393232280231816, -0.25529301514492997, 0.6499788078991758, 0]), 
                 ([4.084804529993941, -0.9423421741448454, 1.2029583913959283], [0.6771127342331346, -0.23599388241935335, -0.6538809053067569, -0.24138142901885606, 0])]
    
    # position_estimates = SimpleRobotPosition.from_list_of_tuple_of_lists(test_list)
    
    # position_estimator = RobotPositionEstimator()
    # estimated_position = position_estimator.estimate_position(position_estimates)
    
    # print(estimated_position)


if __name__ == "__main__":
    test()