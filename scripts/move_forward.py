#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import csv 
import tf
import os

class MoveForward:
    def __init__(self, speed, time, direction, save_location):
        self.speed = speed
        self.time = time
        self.direction = direction
        self.save_location = save_location
        self.movement_completed = False
        
        self.publisher_vel = rospy.Publisher('/nav_vel', Twist, queue_size=1)
        self.publishrate = rospy.Rate(30)
        
        self.tf_listener = tf.TransformListener()
        
        self.amcl_listener = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.record_location_estimate_amcl)
        
        self.estimated_location_tag = None
        self.estimated_location_amcl = None
        
        
    def move(self):
        rospy.loginfo("Started moving")
        start_time = rospy.get_rostime().to_sec()
        
        with open(self.save_location, "w+") as f:
            writer = csv.writer(f)
            header = ["time", "estimated_x", "estimated_y", "amcl_x", "amcl_y"]
        
            while not self.movement_completed:
                self.record_location_estimate_tags()
                
                twist = Twist()
                if self.direction == 'x':
                    twist.linear.x = self.speed
                else:
                    twist.linear.y = self.speed
                
                self.publisher_vel.publish(twist)
                
                row = [rospy.get_time(), self.estimated_location_tag[0], self.estimated_location_tag[1], self.estimated_location_amcl[0], self.estimated_location_amcl[1]]
                writer.writerow(row)
                
                if rospy.get_rostime().to_sec() - start_time > self.time:
                    self.movement_completed = True
                
                self.publishrate.sleep()
            
            twist = Twist()
            self.publisher_vel.publish(twist)
            
            rospy.loginfo("Movement completed")
    
    def record_location_estimate_tags(self):
        """ record the estimate location of the robot according to the tags """
        self.tf_listener.waitForTransform("/map", "/world_location", rospy.Time(), rospy.Duration(0.5))
        estimated_position_tf = self.tf_listener.lookupTransform("/map", "/world_location", rospy.Time(0))
        self.estimated_location_tag = (estimated_position_tf[0][0], estimated_position_tf[0][1])
    
    def record_location_estimate_amcl(self, data):
        """ record the estimated location of the robot according to the amcl """
        self.estimated_location_amcl = (data.pose.pose.position.x, data.pose.pose.position.y)
        
            
if __name__ == '__main__':
    rospy.init_node('move_forward')
    
    speed = rospy.get_param("~speed")
    time = rospy.get_param("~time")
    direction = rospy.get_param("~direction")
    
    save_location = rospy.get_param("~save_location")
    save_name = rospy.get_param("~save_name")
    save_format = rospy.get_param("~save_format")

    try: 
        os.makedirs(save_location)
    except OSError as fail: 
        pass
    
    move_forward = MoveForward(speed, time, direction, save_location + "/" + save_name + save_format)
    
    move_forward.move()
        
        