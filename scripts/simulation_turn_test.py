#!/usr/bin/python
from simulation_straight_test import SimulationMove
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import numpy as np
import os

class MoveAndTurn:
    def __init__(self, time, speed, angle):
        self.time = time
        self.speed = speed
        self.angle = angle
        
    def move(self):
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.angle
        return twist


if __name__ == "__main__":
    
    rospy.init_node('simulation_move_test')
    
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
        pose.pose.pose.position.x = -2.5
        pose.pose.pose.position.y = 1.5
        pose.pose.pose.orientation.w = 1
        pose.pose.pose.orientation.z = 0
        initial_pose_pub.publish(pose)
        i += 1
        rospy.sleep(0.01)

    rospy.loginfo("Set correct initial pose")
    
    rospy.sleep(3)
    
    try: 
        os.makedirs(save_location)
    except OSError as fail: 
        pass
        
    time = 5
    distance = 2
    moves = [MoveAndTurn(5.4, 0.4, -0.3)]
    
    move_square_test = SimulationMove(save_location + "/" + save_name + save_format, moves=moves)
    
    move_square_test.move()