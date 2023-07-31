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
from cv_bridge import CvBridge, CvBridgeError

import asyncio
platform_main_loop = asyncio.get_event_loop()
platform_main_task = set()

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

        self.kick_goal.kick_direction = Quaternion()
        self.kick_goal.kick_direction.x = 0
        self.kick_goal.kick_direction.y = 0
        self.kick_goal.kick_direction.z = 0
        self.kick_goal.kick_direction.w = 1

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
        
        self.image_rgb = []
        
        self.accl_info = [0,0,0]
        self.gyro_info = [0,0,0]
        
        platform_main_task.add(asyncio.create_task(self.imu_subscriber.imu_callback))
        platform_main_task.add(asyncio.create_task(self.img_subscriber.img_callback))

    def bot_token(self):
        return self.token
    
    async def img_callback(self, img_raw): 
        bridge = CvBridge() 
        image = bridge.imgmsg_to_cv2(img_raw, "bgr8")
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        self.image_rgb = image
        cv2.imshow('img3', self.image_rgb)

    async def imu_callback(self, imu_raw):
        self.accl_info = [imu_raw.linear_acceleration.x, imu_raw.linear_acceleration.y, imu_raw.linear_acceleration.z]     
        self.gyro_info = [imu_raw.angular_velocity.x, imu_raw.angular_velocity.y, imu_raw.angular_velocity.z] 
        print(self.token + " accelerometer")
        print(self.accl_info)
        print(self.token + " gyroscope")
        print(self.gyro_info)
        
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
        platform_main_loop.run_until_complete(platform_main_task)
        

        
    
