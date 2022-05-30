#!/usr/bin/python

from logging import shutdown
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import csv 
import tf
import os

class MoveForward:
    def __init__(self, speed, time, direction, save_location, shutdown_time = 1):
        self.speed = speed
        self.time = time
        self.direction = direction
        self.save_location = save_location
        self.movement_completed = False
        self.shutdown_time = shutdown_time
        
        self.publisher_vel = rospy.Publisher('/nav_vel', Twist, queue_size=1)
        self.publishrate = rospy.Rate(30)
        self.saverate = rospy.Rate(30)
        
        self.tf_listener = tf.TransformListener()
        
        # self.amcl_listener = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.record_location_estimate_amcl)
        
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
                self.record_location_estimate_amcl()
                
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
            
            while rospy.get_rostime().to_sec() - start_time < self.time + self.shutdown_time:
                self.record_location_estimate_tags()
                self.record_location_estimate_amcl()
                row = [rospy.get_time(), self.estimated_location_tag[0], self.estimated_location_tag[1], self.estimated_location_amcl[0], self.estimated_location_amcl[1]]
                writer.writerow(row)
                self.saverate.sleep()
            
            rospy.loginfo("Movement completed")
    
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
        
            
if __name__ == '__main__':
    rospy.init_node('move_forward')
    
    
    speed = rospy.get_param("~speed", 0.5)
    time = rospy.get_param("~time", 1)
    direction = rospy.get_param("~direction", "x")
    
    save_location = rospy.get_param("~save_location", "/home/levijn/BEP/simulation_ws/move_forward_tests")
    save_name = rospy.get_param("~save_name", "move_forward_test")
    save_format = rospy.get_param("~save_format", ".csv")

    test_folder_location = os.path.join(save_location, save_name)
    
    rospy.sleep(3)

    try: 
        os.mkdir(save_location)
        os.mkdir(test_folder_location)
    except OSError as fail: 
        try:
            os.mkdir(test_folder_location)
        except OSError as fail2:
            pass
    
    move_forward = MoveForward(speed, time, direction, test_folder_location + "/" + save_name + save_format)
    
    move_forward.move()
        
        