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
        self.msg_thread.start()
            
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
        print("Team " + self.teamside + " is ready!")
        if(self.botnum> 0):
            while True:
                await asyncio.sleep(0)
                for i in range(self.botnum):
                    # test img first
                    if( len(self.bot_sensor)> 0):
                        if(not self.bot_sensor[i].imu_queue.empty()):
                            img_msg = self.bot_sensor[i].img_queue.get()
                            await self.server.push_captured_image(self.bot_sensor[i].token, img_msg)  
                    else:
                        pass               
        else:
            pass
        await self.server.close()    
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--bot_num', type = int, default = None)
    args = parser.parse_args()
    print("------battle platform ver1.0------")
    rospy.init_node('battle_platform', anonymous=True) 
    print("ros node init")
    red_team = battle_manager(args.bot_num, 'r')
    asyncio.run(red_team.sever_thread_run())
        
    
