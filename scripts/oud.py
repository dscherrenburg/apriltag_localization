    
    def moving_avg(self):
        """calculates the moving average of each tag and returns a dicionary of Pose message types"""
        avgs = {}
        for tag_id in self.tag_history:
            tag_pos = self.tag_history[tag_id]
            avg_x = sum([pos.position.x for pos in tag_pos]) / self.moving_avg_len
            avg_y = sum([pos.position.y for pos in tag_pos]) / self.moving_avg_len
            avg_z = sum([pos.position.z for pos in tag_pos]) / self.moving_avg_len
            avg_rot_w = sum([pos.orientation.w for pos in tag_pos]) / self.moving_avg_len
            avg_rot_x = sum([pos.orientation.x for pos in tag_pos]) / self.moving_avg_len
            avg_rot_y = sum([pos.orientation.y for pos in tag_pos]) / self.moving_avg_len
            avg_rot_z = sum([pos.orientation.z for pos in tag_pos]) / self.moving_avg_len

            avg_pose = Pose()
            avg_pose.position.x = avg_x
            avg_pose.position.y = avg_y
            avg_pose.position.z = avg_z
            avg_pose.orientation.w = avg_rot_w
            avg_pose.orientation.x = avg_rot_x
            avg_pose.orientation.y = avg_rot_y
            avg_pose.orientation.z = avg_rot_z

            avgs[tag_id] = avg_pose
        return avgs

    def calculate_robot_pose_per_tag(self, tf_tag_world, tf_tag_robot):
        """Calculates the robot pose relative to the world."""
        tf_world_robot = Pose()
        tf_world_robot.position.x = tf_tag_world.position.x - tf_tag_robot.position.x
        tf_world_robot.position.y = tf_tag_world.position.y - tf_tag_robot.position.y
        tf_world_robot.position.z = tf_tag_world.position.z - tf_tag_robot.position.z

        #rospy.loginfo(tf_world_robot)

    # def detection_callback(self, data):
    #     """Called every time the tag_detection topic is published. Saves the tag_detection message in self.tag_history. 
    #     Then calculates the moving average of each tag.
    #     """

    #     # save the latest relative position of each vissible tag
    #     for tag in data.detections:
    #         id = tag.id[0]
    #         if id not in self.tag_history:
    #             self.tag_history[id] = []
    #         self.tag_history[id].append(tag.pose.pose.pose)
    #         if len(self.tag_history[id]) > self.moving_avg_len:     
    #             self.tag_history[id] = self.tag_history[id][-self.moving_avg_len:]
        
    #     # calculate the moving average
    #     new_moving_avg = self.moving_avg()
    #     # rospy.loginfo(new_moving_avg)
    #     # self.calculate_robot_pose_per_tag(new_moving_avg[0], self.apriltag_poses['Apriltag36_11_00006'])

        

        self.model_states = rospy.wait_for_message('gazebo/model_states', ModelStates)
        self.apriltag_poses = {'tag_' + str(int(name[-3:])) : self.model_states.pose[i] for i, name in enumerate(self.model_states.name) if 'Apriltag' in name}