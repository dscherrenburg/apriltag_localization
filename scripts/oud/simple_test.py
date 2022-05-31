#!/usr/bin/python

import rospy
import tf
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
import csv
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from plot_test import create_plots, all_plots
import os


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
    def __init__(self, robot_name, write_rate, move_sequence, save_location):
        self.test_completed = False
        self.robot_name = robot_name
        self.write_rate = rospy.Rate(write_rate)
        self.move_sequence = move_sequence
        self.save_location = save_location
        
        self.tf_listener = tf.TransformListener()
        self.gazebo_states_listener = rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_states_callback)
        
        self.true_location = None
        self.estimated_location = None
        
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()


    def run_test(self):
        # self.set_robot_state(0, 0, 0)
        
        with open(self.save_location, "w+") as f:
            writer = csv.writer(f)
            
            header = ["time", "true_x", "true_y", "estimated_x", "estimated_y"]
            writer.writerow(header)

            for pos in self.move_sequence:
                next_goal = self.get_goal(pos[0], pos[1], pos[2])
                self.move_base_client.send_goal(next_goal)
                rospy.loginfo(self.move_base_client.get_state())
                while self.move_base_client.get_state() == (0 or 1):
                    self.record_location_estimate()
                    row = [rospy.get_time(), self.true_location[0], self.true_location[1], self.estimated_location[0], self.estimated_location[1]]
                    writer.writerow(row)
                    rospy.loginfo("wrote row: %s", row)
                    self.write_rate.sleep()
            
            
    def set_robot_state(self, x, y, r):
        # latest_message = self.gazebo_states_listener.get_latest_message()
        # i = latest_message.name.index(self.robot_name)
        # latest_position = latest_message.pose[i]
        state_msg = ModelState()
        state_msg.model_name = self.robot_name
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = 0
        
        q = quaternion_from_euler(0, 0, r)
        
        state_msg.pose.orientation.x = q[1]
        state_msg.pose.orientation.y = q[2]
        state_msg.pose.orientation.z = q[3]
        state_msg.pose.orientation.w = q[0]
        
        set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        set_state(state_msg)

    def get_goal(self, x, y, r):
        """ start the movement of the robot """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "base_link"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        q = quaternion_from_euler(0, 0, r)
        goal.target_pose.pose.orientation.x = q[1]
        goal.target_pose.pose.orientation.y = q[2]
        goal.target_pose.pose.orientation.z = q[3]
        goal.target_pose.pose.orientation.w = q[0]
        
        return goal

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
    
    move_sequence = [(2, 0, 0), (2, 0, 0)]
    
    save_name = rospy.get_param("~test_file_name")
    save_location = rospy.get_param("~test_file_location")
    save_format = rospy.get_param("~test_file_format")
    rospy.loginfo(save_location)

    try: 
        os.makedirs(save_location)
    except OSError as fail: 
        pass
    
    test = SimpleTest("tiago_dual", 20, move_sequence, save_location+"/"+save_name+save_format)
    test.run_test()

    # create_plots(test_folder_location, save_name, save_format)