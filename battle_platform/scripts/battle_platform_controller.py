#!/usr/bin/env python3

import math
import os
import rospy
import argparse

import sys
import asyncio
import datetime
import pathlib
import actionlib

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent.resolve()))
import src.moshumanoid.sdk as sdk

from geometry_msgs.msg import Twist, Vector3, Quaternion
from sensor_msgs.msg import JointState, Image, Imu
from bitbots_msgs.msg import JointCommand, KickGoal, KickAction, KickFeedback

class battle_player:
    async def __init__(self, bot_side, bot_index):
        self.token = self.side + str(self.index)

        self.walk_goal_publisher = rospy.Publisher(self.token + '/cmd_vel', JointState, queue_size=1)
        self.head_goal_publisher = rospy.Publisher(self.token + '/walking_motor_goals', JointState, queue_size=1)
        self.imu_subscriber = rospy.Subscriber(self.token + '/imu/data_raw', Imu, self.imu_callback, queue_size=1, tcp_nodelay=True)
        self.img_subscriber = rospy.Subscriber(self.token + '/usb_cam/image_raw', Image, self.img_callback, queue_size=1, tcp_nodelay=True)
        
        self.head_goal_msg = JointCommand()
        self.head_goal_msg.joint_names = ["neck", "head"]
        self.head_goal_msg.velocities = [np.pi, np.pi]
        
        self.walk_goal_msg = Twist()
    
    def bot_token(self)
        return self.token
    
    def img_callback(self, img_raw):


    def imu_callback(self, imu_raw):
        self.accelerometer_info = [imu_raw.linear_acceleration.x, imu_raw.linear_acceleration.y, imu_raw.linear_acceleration.z]   
        self.gyroscope_info = [imu_raw.angular_velocity.x, imu_raw.angular_velocity.y, imu_raw.angular_velocity.z] 
        
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
        head_goal.positions = [head_goal_sim[0], head_goal_sim[1]]
        self.head_goal_publisher.publish(head_goal)  
        
    def pub_kick_goal(self, kick_goal, kick_leg, kick_speed = 1, wait_time_before_kick = 3): 
        '''
        kick_goal = [x,y,z] (double)
        kick_leg (bool) - True:left False:right
        kick_speed (double)
        wait_time_before_kick (double)
        '''
        
    def walk_http_callback(self, walk_goal_http):
    
    def kick_http_callback(self, head_goal_http):
    
    def head_http_callback(self, kick_goal_http):   


class battle_sever:
    def __init__(self, bot_num):
        self.red_bots = []
        self.blue_bots = []
        self.token_list = []
        if(bot_num > 0):
            for i in range(args.bot_num):
                self.red_bots.append( battle_player('r', i))
                self.blue_bots.append(battle_player('b', i))
                self.token_list.append( self.red_bots[i].bot_token())
                self.token_list.append(self.blue_bots[i].bot_token())
        else:
            pass
        server = sdk.HttpServer(14514, self.token_list)  
        
    async def main():
        await server.register_callback(callback)
        await server.start()
        '''??????'''
        await server.stop()

    async def callback(token: str, msg: sdk.Message) -> None:
        print(f'{msg["id"]} from {token}')
        await server.send(sdk.Message(
            {
                "type": "",
                "bound_to": "client",
                "id": msg["id"],
            }
        ), token)
         
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--bot_num', type=int, default = None)
    args = parser.parse_args()
    
    rospy.loginfo("------battle platform ver1.0------")
    rospy.init_node('battle_platform', anonymous=True)
    
    bot_sever = battle_sever(args.bot_num)
    asyncio.run(bot_sever.main())

        

