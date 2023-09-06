#!/usr/bin/env python3
import argparse
import rospy
import threading
import asyncio
import datetime
import numpy as np
import soccerxcomm as sdk
import time

from battle_player import battle_sensor, battle_mover

class battle_manager:
    def __init__(self, bot_num, team_side ='r', port_c = 14516, port_t = 14517):
        self.botnum = bot_num
        self.teamside = team_side
        self.portc = port_c
        self.portt = port_t
        
        self.bot_sensor = []
        self.bot_mover = []
        self.token_dict = {}
                
        # status msg
        msg_head_angle = 0.0
        msg_neck_angle = 0.0
        msg_acceleration  = np.array([0.0,0.0,0.0]) 
        msg_angular_velocity = np.array([0.0,0.0,0.0]) 
        msg_attitude_angle = np.array([0.0,0.0,0.0])   
        msg_team = team_side     
        self.bot_status_msg = sdk.RobotStatus(msg_head_angle, 
                                              msg_neck_angle,
                                              msg_acceleration,
                                              msg_angular_velocity,
                                              msg_attitude_angle,
                                              msg_team)
                                              
        self.bot_control_msg = sdk.RobotControl()
        
        if(self.botnum > 0):
            for i in range(self.botnum):
                self.bot_mover.append(battle_mover(self.teamside, i + 1))
                self.token_dict[self.bot_mover[i].token] = sdk.Server.ClientInfo(team = self.teamside, token = self.bot_mover[i].token)
        else:
            pass
            
        self.server = sdk.Server(self.portc, self.portt, self.token_dict)            
        self.msg_thread = threading.Thread(target = self.sensor_thread_run)
        self.msg_thread.start()
        self.count = 0
        
            
    def sensor_thread_run(self):
        if(self.botnum > 0):
            for i in range(self.botnum):
                self.bot_sensor.append( battle_sensor(self.teamside, i + 1))
        else:
            pass
        print('team ' + self.teamside + ' robot sensors are ready')
        rospy.spin()        
    
    
    async def sever_thread_run(self):
        print('waiting for sensor threading start')
        time.sleep(0.5)
        
        await self.server.start()
        print('team ' + self.teamside + ' server is ready')
        
        await self.server.register_robot_control_callback(self.client_callback_run)
        print('team ' + self.teamside + ' server callback is ready')
        
        
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
                            self.bot_status_msg.acceleration = imu_msg[0]
                            self.bot_status_msg.angular_velocity  = imu_msg[1]
                            await self.server.push_robot_status(self.bot_sensor[i].token, self.bot_status_msg)  
                    else:
                        pass               
        else:
            pass
        await self.server.close() 
        
    async def client_callback_run(self, msg_token, bot_ctl_msg):
        self.bot_control_msg = bot_ctl_msg
        print('recieve msg from player ' + msg_token)

        try:
            if (msg_token[0] == self.teamside):
                player_index = int(msg_token[1]) - 1
            else:
                return

            if(self.bot_control_msg.movement != None):
                self.bot_mover[player_index].pub_walk_goal([self.bot_control_msg.movement.x, 
                                                            self.bot_control_msg.movement.y,
                                                            self.bot_control_msg.movement.omega_z])
                                 
            if(self.bot_control_msg.head != None):
                self.bot_mover[player_index].pub_head_goal([self.bot_control_msg.head.neck_angle, 
                                                            self.bot_control_msg.head.head_angle])
                                                          
            if(self.bot_control_msg.kick != None):
                self.bot_mover[player_index].pub_kick_goal([self.bot_control_msg.kick.x, 
                                                            self.bot_control_msg.kick.y,
                                                            self.bot_control_msg.kick.z,
                                                            self.bot_control_msg.kick.speed,
                                                            self.bot_control_msg.kick.delay,])
        except Exception as error:
            print(error)    
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--bot_num', type = int, default = None)
    args = parser.parse_args()
    print("------battle platform ver1.0------")
    rospy.init_node('battle_platform', anonymous=True) 
    red_team = battle_manager(args.bot_num, 'b')
    asyncio.run(red_team.sever_thread_run())
