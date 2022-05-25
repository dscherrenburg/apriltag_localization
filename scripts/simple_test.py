#!/usr/bin/python

import rospy
import tf
from gazebo_msgs.msg import ModelStates
import csv
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import threading


def movebase_client(x, y, r):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.x = x
    
    q = quaternion_from_euler(0, 0, r)
    goal.target_pose.pose.orientation.x = q[1]
    goal.target_pose.pose.orientation.y = q[2]
    goal.target_pose.pose.orientation.z = q[3]
    goal.target_pose.pose.orientation.w = q[0]

    client.send_goal(goal)
    wait = client.wait_for_result()
	
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()



class SimpleTest:
    def __init__(self, robot_name, write_rate, move_sequence):
        self.test_completed = False
        self.robot_name = robot_name
        self.write_rate = rospy.Rate(write_rate)
        
        self.tf_listener = tf.TransformListener()
        self.gazebo_states_listener = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_states_callback)
        
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        
        self.true_location = None
        self.estimated_location = None


    def run_test(self):
        self.start_movement()
        
        with open("simple_test.csv", "w") as f:
            writer = csv.writer(f)
            
            header = ["time", "true_x", "true_y", "estimated_x", "estimated_y"]
            writer.writerow(header)
            
            while not self.test_completed:
                self.record_location_estimate()
                row = [rospy.get_rostime().to_sec(), self.true_location[0], self.true_location[1], self.estimated_location[0], self.estimated_location[1]]
                writer.writerow(row)
                rospy.loginfo("wrote row: %s", row)
                self.write_rate.sleep()


    def start_movement(self):
        """ start the movement of the robot """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.y = 0.0
        goal.target_pose.pose.position.x = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        self.move_base_client.send_goal(goal)


    def record_location_estimate(self):
        """ record the current location of the robot and the estimate location of the robot """
        self.tf_listener.waitForTransform("/map", "/world_location", rospy.Time(), rospy.Duration(0.5))
        estimated_position_tf = self.tf_listener.lookupTransform("/map", "/world_location", rospy.Time(0))
        self.estimated_location = (estimated_position_tf[0][0], estimated_position_tf[0][1])
        
    def gazebo_states_callback(self, data):
        """ callback for the gazebo states. filter out the robot posiiton """
        i = data.name.index(self.robot_name)
        robot_state = data.pose[i]
        self.true_location = (robot_state.position.x, robot_state.position.y)
        

    

if __name__ == '__main__':
    rospy.init_node('simple_test')
    test = SimpleTest("tiago_dual", 5, [(1, 0, 0), (0, 1, 0)])
    while not rospy.is_shutdown():
        test.run_test()
        rospy.spin