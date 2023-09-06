#!/usr/bin/env python3
import asyncio
import cv2
import soccerxcomm as sdk
import random
async def main():
    client = sdk.Client("localhost", 14514, 14515, "r1")

    await client.connect()

    await asyncio.sleep(2)
    
    # HeadMove
    print('head')
    ctl_msg = sdk.RobotControl(head = sdk.RobotControl.Head(random.random(),random.random()))
    await client.push_robot_control(ctl_msg)

    # WalkMove
    print('walk')
    ctl_msg = sdk.RobotControl(movement = sdk.RobotControl.Movement(0.01,0.0,0.0))
    await client.push_robot_control(ctl_msg)
    await asyncio.sleep(2)
    ctl_msg = sdk.RobotControl(movement = sdk.RobotControl.Movement(0.0,0.0,0.0))
    await client.push_robot_control(ctl_msg)

    # KickMove
    print('kick')
    ctl_msg = sdk.RobotControl(kick = sdk.RobotControl.Kick(0.1,0.0,0.0,0.3,1.0))
    await client.push_robot_control(ctl_msg)
    
    # ImuTest
    rec_msg = await client.get_robot_status()
    print('imu info')
    print('acceleration ', rec_msg.acceleration)
    print('angular velocity ',rec_msg.angular_velocity)

    # ImageTest
    captured_image = await client.get_captured_image()
    print('captured_image')
    cv2.imshow('img',captured_image)
    cv2.waitKey(0)

    await client.disconnect()

if __name__ == '__main__':
    asyncio.run(main())
    print("done")

