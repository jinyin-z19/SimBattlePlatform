#!/usr/bin/env python3
import argparse
import rospy
import threading
import asyncio
import datetime
import numpy as np
import soccerxcomm as sdk

from battle_player import battle_sensor, battle_mover
from queue import Queue,LifoQueue

class battle_manager:
    def __init__(self, bot_num, team_side ='r', port_c = 14514, port_t = 14515):
        self.botnum = bot_num
        self.teamside = team_side
        self.portc = port_c
        self.portt = port_t
        
        self.bot_sensor = []
        self.bot_mover = []
        
        self.bot_control_msg = sdk.RobotControl()
        self.bot_status_msg = sdk.RobotStatus()
        
        # delete useless msg
        self.bot_status_msg.head_angle = 0
        self.bot_status_msg.neck_angle = 0
        self.bot_status_msg.attitude_angle = np.array([0,0,0])       
        
        self.token_dict = {}
        
        if(self.botnum > 0):
            for i in range(self.botnum):
                self.bot_mover.append(battle_mover(self.teamside, i + 1))
                print(self.bot_mover[i].token)
                self.token_dict[self.bot_mover[i].token] = sdk.Server.ClientInfo(team = self.teamside, token = self.bot_mover[i].token)
        else:
            pass
            
        print("Team " + self.teamside + " mover is ready!")
        self.server = sdk.Server(self.portc, self.portt, self.token_dict)            
        self.msg_thread = threading.Thread(target = self.sensor_thread_run)
        self.ctl_thread = threading.Thread(target = self.sensor_thread_run)
        self.msg_thread.start()
        self.ctl_thread.start()
        
        self.sever.register_robot_control_callback(self.client_callback_run)
        
            
    def sensor_thread_run(self):
        if(self.botnum > 0):
            for i in range(self.botnum):
                self.bot_sensor.append( battle_sensor(self.teamside, i + 1))
        else:
            pass
        print("Team " + self.teamside + " sensor is ready!")
        rospy.spin()        
    
    
    async def sever_thread_run(self):
        await self.server.start()
        print("Team " + self.teamside + " sever is ready!")
        if(self.botnum> 0):
            while True:
                await asyncio.sleep(0)
                for i in range(self.botnum):
                    if( len(self.bot_sensor)> 0):
                    
                        # img msg send
                        if(not self.bot_sensor[i].img_queue.empty()):
                            img_msg = self.bot_sensor[i].img_queue.get()
                            await self.server.push_captured_image(self.bot_sensor[i].token, img_msg)
                              
                        # imu msg send
                        if(not self.bot_sensor[i].imu_queue.empty()):
                            imu_msg = self.bot_sensor[i].imu_queue.get()
                            self.bot_status_msg.acceleration = imu_msg[0:3]
                            self.bot_status_msg.angular_velocity  = imu_msg[3:6]
                            await self.server.push_robot_status(self.bot_sensor[i].token, imu_msg)  
                    else:
                        pass               
        else:
            pass
        await self.server.close() 
        
    def client_callback_run(self, msg_token, bot_ctl_msg):
        self.bot_control_msg = bot_ctl_msg
        if (msg_token[0] == self.teamside):
            player_index = int(msg_token[0])
        else:
            return
         
        if(self.bot_control_msg.movement != None):
            self.bot_mover[player_index].pub_walk_goal([self.bot_control_msg.movement.x, 
                                                        self.bot_control_msg.movement.y,
                                                        self.bot_control_msg.movement.omega_z])
                                                          
        if(self.bot_control_msg.head != None):
             self.bot_mover[player_index].pub_head_goal([self.bot_control_msg.head.neck_angle, 
                                                         self.bot_control_msg.head.hea_angle])
                                                          
        if(self.bot_control_msg.kick != None):
            self.bot_mover[player_index].pub_kick_goal([self.bot_control_msg.kick.x, 
                                                        self.bot_control_msg.kick.y,
                                                        self.bot_control_msg.kick.z,
                                                        self.bot_control_msg.kick.speed,
                                                        self.bot_control_msg.kick.delay,])         
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--bot_num', type = int, default = None)
    args = parser.parse_args()
    print("------battle platform ver1.0------")
    rospy.init_node('battle_platform', anonymous=True) 
    print("ros node init")
    red_team = battle_manager(args.bot_num, 'r')
    asyncio.run(red_team.sever_thread_run())
