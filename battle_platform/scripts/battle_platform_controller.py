#!/usr/bin/env python3

import math
import os
import rospy
import argparse
import actionlib

from geometry_msgs.msg import Twist, Vector3, Quaternion
from sensor_msgs.msg import JointState, Image, Imu
from bitbots_msgs.msg import JointCommand, KickGoal, KickAction, KickFeedback


class robot_kick:
    def __init__(self, token):
        self.kick_goal = KickGoal()
        self.kick_client = actionlib.SimpleActionClient(token + '/dynamic_kick', KickAction)
        self.kick_client.wait_for_server()
    def done_cb(self, state, result):
        rospy.loginfo("kick action complete")

    def active_cb(self):
        rospy.loginfo('kick server accepted')

    def feedback_cb(self, feedback):
        rospy.loginfo(feedback)   
      
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

        kick_client.send_goal(self.kick_goal)
        rospy.loginfo("send kick")

        self.kick_client.done_cb = done_cb
        #self.kick_client.feedback_cb = feedback_cb
        self.kick_client.active_cb = active_cb
        self.kick_client.wait_for_result()
        
        rospy.loginfo("kick down")   
    
class battle_player:
    def __init__(self, bot_side, bot_index):
        self.token = self.side + str(self.index)

        self.walk_goal_publisher = rospy.Publisher(self.token + '/cmd_vel', JointState, queue_size=1)
        self.head_goal_publisher = rospy.Publisher(self.token + '/walking_motor_goals', JointState, queue_size=1)
        self.imu_subscriber = rospy.Subscriber(self.token + '/imu/data_raw', Imu, self.imu_callback, queue_size=1, tcp_nodelay=True)
        self.img_subscriber = rospy.Subscriber(self.token + '/usb_cam/image_raw', Image, self.img_callback, queue_size=1, tcp_nodelay=True)
        
        self.bot_kick = robot_kick(self.token)
        
        self.head_goal_msg = JointCommand()
        self.head_goal_msg.joint_names = ["neck", "head"]
        self.head_goal_msg.velocities = [np.pi, np.pi]
        
        self.walk_goal_msg = Twist()
        
        self.accl_info = [0,0,0]
        self.gyro_info = [0,0,0]
    
    def bot_token(self)
        return self.token
    
    def img_callback(self, img_raw):
        return img_raw

    def imu_callback(self, imu_raw):
        accelerometer_info = [imu_raw.linear_acceleration.x, imu_raw.linear_acceleration.y, imu_raw.linear_acceleration.z] 
        self.accl_info =  accelerometer_info      
        gyroscope_info = [imu_raw.angular_velocity.x, imu_raw.angular_velocity.y, imu_raw.angular_velocity.z] 
        self.gyro_info =  gyroscope_info 
        return accelerometer_info, gyroscope_info
        
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
         
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--bot_num', type = int, default = None)
    args = parser.parse_args()
    
    rospy.loginfo("------battle platform ver1.0------")
    rospy.init_node('battle_platform', anonymous=True)
    
    red_bots = []
    blue_bots = []
    token_list = []
    if(bot_num > 0):
        for i in range(args.bot_num):
            red_bots.append( battle_player('r', i))
            blue_bots.append(battle_player('b', i))
            token_list.append( self.red_bots[i].bot_token())
            token_list.append(self.blue_bots[i].bot_token())
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
        # kick example
        red_bots[0].pub_kick_goal([0.3,0.05,0])
        

        
    
