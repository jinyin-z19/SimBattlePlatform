#!/usr/bin/env python

import math
import time
import os
import rospy
import argparse
import actionlib
import cv2
import numpy 

from geometry_msgs.msg import Twist, Vector3, Quaternion
from sensor_msgs.msg import JointState, Image, Imu
from bitbots_msgs.msg import JointCommand, KickGoal, KickAction, KickFeedback
from tf.transformations import quaternion_from_euler
from cv_bridge import CvBridge, CvBridgeError

class robot_kick:
    def __init__(self, token):
        self.kick_goal = KickGoal()
        self.kick_client = actionlib.SimpleActionClient(token + '/dynamic_kick', KickAction)
        self.kick_client.wait_for_server()
      
    def kick(self, kick_pos, kick_speed, wait_time_before_kick):
        self.kick_goal.header.seq = 1
        self.kick_goal.header.stamp = rospy.Time.now()
        frame_prefix = "" if os.environ.get("ROS_NAMESPACE") is None else os.environ.get("ROS_NAMESPACE") + "/"
        self.kick_goal.header.frame_id = frame_prefix + "base_footprint"

     
        self.kick_goal.ball_position.x = kick_pos[0]
        self.kick_goal.ball_position.y = kick_pos[1]
        self.kick_goal.ball_position.z = kick_pos[2]

        self.kick_goal.kick_direction = Quaternion(*quaternion_from_euler(0,0,0))

        self.kick_goal.kick_speed = kick_speed

        rospy.loginfo("kick goal init")

        rospy.sleep(wait_time_before_kick)

        self.kick_client.send_goal(self.kick_goal)
        rospy.loginfo("send kick")

        self.kick_client.wait_for_result()
        
        rospy.loginfo("kick down")   
    
class battle_player:
    def __init__(self, bot_side, bot_index):
        self.token = bot_side + str(bot_index)

        self.walk_goal_publisher = rospy.Publisher(self.token + '/cmd_vel', Twist, queue_size=1)
        self.head_goal_publisher = rospy.Publisher(self.token + '/walking_motor_goals', JointCommand, queue_size=1)
        self.imu_subscriber = rospy.Subscriber(self.token + '/imu/data_raw', Imu, self.imu_callback, queue_size=1, tcp_nodelay=True)
        self.img_subscriber = rospy.Subscriber(self.token + '/usb_cam/image_raw', Image, self.img_callback, queue_size=1, tcp_nodelay=True)
        
        self.bot_kick = robot_kick(self.token)
        
        self.head_goal_msg = JointCommand()
        self.head_goal_msg.joint_names = ["neck", "head"]
        self.head_goal_msg.velocities = [3.1415926, 3.1415926]
        
        self.walk_goal_msg = Twist()
        
        self.image_raw = Image()
        
        self.accl_info = [0,0,0]
        self.gyro_info = [0,0,0]
    
    def bot_token(self):
        return self.token
    
    def img_callback(self, img_raw):
        self.image_raw = img_raw

    def imu_callback(self, imu_raw):
        self.accl_info = [imu_raw.linear_acceleration.x, imu_raw.linear_acceleration.y, imu_raw.linear_acceleration.z]     
        self.gyro_info = [imu_raw.angular_velocity.x, imu_raw.angular_velocity.y, imu_raw.angular_velocity.z] 
        
    def pub_walk_goal(self, walk_goal_sim):
        '''
        walk_goal_sim = [vx,vy,omega_z] (double)
        '''
        self.walk_goal_msg.linear.x = walk_goal_sim[0]
        self.walk_goal_msg.linear.y = walk_goal_sim[1]
        self.walk_goal_msg.angular.z = walk_goal_sim[2] 
        self.walk_goal_publisher.publish(self.walk_goal_msg)    
        
    def pub_head_goal(self, head_goal_sim):
        '''
        head_goal_sim = [neck_angle, head_angle] (double)
        '''
        self.head_goal_msg.positions = [head_goal_sim[0], head_goal_sim[1]]
        self.head_goal_publisher.publish(self.head_goal_msg)  
        
    def pub_kick_goal(self, kick_pos, kick_speed = 1, wait_time_before_kick = 3): 
        '''
        kick_pos = [x,y,z] (double)
        kick_speed (double)
        wait_time_before_kick (double)
        '''
        self.bot_kick.kick(kick_pos, kick_speed, wait_time_before_kick) 

    def get_latest_imu(self): 
        return self.accl_info, self.gyro_info

    def get_latest_img(self):
        bridge = CvBridge() 
        image = bridge.imgmsg_to_cv2(self.image_raw, "bgr8")
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return image  

#-----------example-------------
     
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--bot_num', type = int, default = None)
    args = parser.parse_args()
    
    rospy.loginfo("------battle platform ver1.0------")
    rospy.init_node('battle_platform', anonymous=True)
    
    red_bots = []
    blue_bots = []
    token_list = []
    if(args.bot_num > 0):
        for i in range(args.bot_num):
            red_bots.append( battle_player('r', i + 1))
            blue_bots.append(battle_player('b', i + 1))
            token_list.append( red_bots[i].bot_token())
            token_list.append(blue_bots[i].bot_token())
    else:
        pass
        
    print(token_list)
    
    while not rospy.is_shutdown():

        # walk example
        red_bots[0].pub_walk_goal([0.1,0,0])
        time.sleep(3)

        red_bots[0].pub_walk_goal([0,0,0])
        time.sleep(0.5)

        # head example
        red_bots[0].pub_head_goal([0.7,0.7])
        time.sleep(3)

        red_bots[0].pub_head_goal([0,0])
        time.sleep(0.5)

        # read photo example
        img_r1 = red_bots[0].get_latest_img()
        cv2.imshow('img3', img_r1)
        cv2.waitKey(0)

        # read imu example
        accl_r1, gyro_r1 = red_bots[0].get_latest_imu()
        print("accelerometer")
        print(accl_r1)
        print("gyroscope")
        print(gyro_r1)

        # kick example
        red_bots[0].pub_kick_goal([0.3,0.05,0])

        

        
    
