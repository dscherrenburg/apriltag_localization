#!/usr/bin/python
import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import csv 
import tf
import os
from gazebo_msgs.msg import ModelStates


class Move:
    def __init__(self, time, speed):
        self.time = time
        self.speed = speed
        
    def move(self):
        twist = Twist()
        twist.linear.x = self.speed
        return twist
        

class Rotate:
    def __init__(self, time, angle):
        self.time = time
        self.angle = angle
    
    def move(self):
        twist = Twist()
        twist.angular.z = self.angle
        return twist
        

class SimulationMove:
    def __init__(self, save_location, shutdown_time = 1, moves = []):
        self.save_location = save_location
        self.movement_completed = False
        self.shutdown_time = shutdown_time
        self.moves = moves
        self.robot_name = "tiago_dual"
        
        self.publisher_vel = rospy.Publisher('/nav_vel', Twist, queue_size=1)
        self.publish_rate = rospy.Rate(30)
        
        self.tf_listener = tf.TransformListener()
        self.gazebo_states_listener = rospy.Subscriber("/gazebo/model_states", ModelStates, self.record_true_location)
        
        self.estimated_location_tag = None
        self.estimated_location_amcl = None
        self.true_location = None
        
        
    def move(self):
        with open(self.save_location, 'w+') as f:
            writer = csv.writer(f)
            header = ["time", "true_location_x", "true_location_y", "estimated_location_tag_x", "estimated_location_tag_y", "estimated_location_amcl_x", "estimated_location_amcl_y"]
            writer.writerow(header)
            
            rospy.loginfo("Started moving")
            start_time = rospy.get_rostime().to_sec()
            
            for move in self.moves:
                while start_time + move.time > rospy.get_rostime().to_sec():
                    self.record_location_estimate_amcl()
                    self.record_location_estimate_tags()
                    
                    row = [rospy.get_time(), self.true_location[0], self.true_location[1], self.estimated_location_tag[0], self.estimated_location_tag[1], self.estimated_location_amcl[0], self.estimated_location_amcl[1]]
                    writer.writerow(row)
                    
                    self.publisher_vel.publish(move.move())
                    
                    self.publish_rate.sleep()
                    
                rospy.sleep(1)
                start_time = rospy.get_rostime().to_sec()
            
            rospy.loginfo("Finished moving")
            
            while start_time + 0.5 > rospy.get_rostime().to_sec():
                self.record_location_estimate_amcl()
                self.record_location_estimate_tags()
                
                row = [rospy.get_time(), self.true_location[0], self.true_location[1], self.estimated_location_tag[0], self.estimated_location_tag[1], self.estimated_location_amcl[0], self.estimated_location_amcl[1]]
                writer.writerow(row)                
                self.publish_rate.sleep()
            
            rospy.loginfo("Finished recording")
    
    
    def record_location_estimate_tags(self):
        """ record the estimate location of the robot according to the tags """
        self.tf_listener.waitForTransform("/map", "/world_location", rospy.Time(), rospy.Duration(0.5))
        estimated_position_tf = self.tf_listener.lookupTransform("/map", "/world_location", rospy.Time(0))
        self.estimated_location_tag = (estimated_position_tf[0][0], estimated_position_tf[0][1])
    
    def record_location_estimate_amcl(self):
        """ record the estimated location of the robot according to the amcl """
        self.tf_listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(0.5))
        estimated_position_tf = self.tf_listener.lookupTransform("/map", "/base_link", rospy.Time(0))
        self.estimated_location_amcl = (estimated_position_tf[0][0], estimated_position_tf[0][1])
        
    def record_true_location(self, data):
        """ callback for the gazebo states. filter out the robot posiiton """
        i = data.name.index(self.robot_name)
        robot_state = data.pose[i]
        self.true_location = (robot_state.position.x, robot_state.position.y)
        
            
if __name__ == '__main__':
    rospy.init_node('move_square_test')
    
    save_location = rospy.get_param("~save_location")
    save_name = rospy.get_param("~save_name")
    save_format = rospy.get_param("~save_format")
    
    rospy.loginfo("Wait to start movement node")
    rospy.sleep(4)
    
    initial_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
    
    rospy.loginfo("Setting initial pose")
    
    i = 0
    while i < 30:
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.get_rostime()
        pose.pose.pose.position.x = -1
        pose.pose.pose.position.y = 2
        pose.pose.pose.orientation.w = 0.7071408
        pose.pose.pose.orientation.z = -0.7070727
        initial_pose_pub.publish(pose)
        i += 1
        rospy.sleep(0.01)

    rospy.loginfo("Set correct initial pose")
    
    rospy.sleep(1)
    
    try: 
        os.makedirs(save_location)
    except OSError as fail: 
        pass
        
    moves = [Move(4, 0.25)]
    
    move_square_test = SimulationMove(save_location + "/" + save_name + save_format, moves=moves)
    
    move_square_test.move()