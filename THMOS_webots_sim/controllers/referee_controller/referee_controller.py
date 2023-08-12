#!/usr/bin/env python3

from controller import Supervisor
import rospy
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String
# import tf_conversions
import numpy as np

class RobotController:
    def __init__(self):
        self.robot_node = Supervisor()
        self.time = 0
        self.timestep = int(self.robot_node.getBasicTimeStep())
 
        rospy.init_node("referee_webots_interface")

        self.pub_robot_pose_1 = rospy.Publisher("r1/robot_pose", Pose, queue_size=1)
        self.pub_robot_pose_2 = rospy.Publisher("b1/robot_pose", Pose, queue_size=1)
        self.pub_soccer_pose = rospy.Publisher("soccer_position", Point, queue_size=1)
        rospy.Subscriber("/process_control", String, self.command_process_control, queue_size=1)

    def step_sim(self):
        self.time += self.timestep / 1000
        self.robot_node.step(self.timestep)

    def step(self):
        self.step_sim()
        self.publish_ros()

    def publish_ros(self):
        self.publish_robot_pose()
        self.publish_soccer_pose()


    def command_process_control(self,command: String):
        print("Received")
        if command.data == 'Pause':
            print("Simulation Pause")
            # self.robot_node.robot_step(0)
            self.robot_node.simulationSetMode(self.robot_node.SIMULATION_MODE_PAUSE)
            # time.sleep(5)
            # self.robot_node.step(0)
            # self.robot_node.simulationSetMode(self.robot_node.SIMULATION_MODE_REAL_TIME)
        elif command.data == 'Reset':
            print("Simulation Reset")
            self.robot_node.simulationReset()
        # elif command.data == 'Run':
        #     print("Simulation Run")
        #     self.robot_node.step(0)
        #     self.robot_node.simulationSetMode(self.robot_node.SIMULATION_MODE_REAL_TIME)

    def get_pose_msg_1(self):
        pose_msg= Pose()
        # pose_msg.header.stamp = rospy.Time.from_seconds(self.time)
        # pose_msg.header.frame_id = self.pos_frame
        robot1_node = self.robot_node.getFromDef("red_player_1")
        position = robot1_node.getPosition()
        orientation_matrix = robot1_node.getOrientation()
        # orientation_matrix = orientation_matrix.reshape(3,3)
        orientation_matrix = np.array([[orientation_matrix[0], orientation_matrix[1], orientation_matrix[2], 0],
                                                                        [orientation_matrix[3], orientation_matrix[4], orientation_matrix[5], 0],
                                                                        [orientation_matrix[6], orientation_matrix[7], orientation_matrix[8], 0],
                                                                        [0, 0, 0, 1] ])
        # quaternion_from_matrix needs a 4x4 marix, and donot use the 4st colum
        # default seq is xyzw
        # orientation = tf_conversions.transformations.quaternion_from_matrix(orientation_matrix)
        # print(orientation)
        # Extract the rotation matrix and remove the last column and row
        rotation_matrix = orientation_matrix[:3, :3]

        # Calculate the quaternion from the rotation matrix
        w = np.sqrt(1 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]) / 2
        x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4 * w)
        y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4 * w)
        z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4 * w)

        orientation = np.array([x, y, z, w])
        # print('numpy')
        # print(orientation)
        pose_msg.position.x = position[0]
        pose_msg.position.y = position[1]
        pose_msg.position.z = position[2]

        pose_msg.orientation.x = orientation[0]
        pose_msg.orientation.y = orientation[1]
        pose_msg.orientation.z = orientation[2]
        pose_msg.orientation.w = orientation[3]

        # print(tf_conversions.transformations.quaternion_matrix(orientation))
        # print(orientation_matrix)
        return pose_msg
    
    def get_pose_msg_2(self):
        pose_msg= Pose()
        # pose_msg.header.stamp = rospy.Time.from_seconds(self.time)
        # pose_msg.header.frame_id = self.pos_frame
        robot2_node = self.robot_node.getFromDef("blue_player_1")
        position = robot2_node.getPosition()
        orientation_matrix = robot2_node.getOrientation()
        # orientation_matrix = orientation_matrix.reshape(3,3)
        orientation_matrix = np.array([[orientation_matrix[0], orientation_matrix[1], orientation_matrix[2], 0],
                                                                        [orientation_matrix[3], orientation_matrix[4], orientation_matrix[5], 0],
                                                                        [orientation_matrix[6], orientation_matrix[7], orientation_matrix[8], 0],
                                                                        [0, 0, 0, 1] ])
        # quaternion_from_matrix needs a 4x4 marix, and donot use the 4st colum
        # default seq is xyzw
        # orientation = tf_conversions.transformations.quaternion_from_matrix(orientation_matrix)
        # print(orientation)
        # Extract the rotation matrix and remove the last column and row
        rotation_matrix = orientation_matrix[:3, :3]

        # Calculate the quaternion from the rotation matrix
        w = np.sqrt(1 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]) / 2
        x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4 * w)
        y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4 * w)
        z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4 * w)

        orientation = np.array([x, y, z, w])
        # print('numpy')
        # print(orientation)
        pose_msg.position.x = position[0]
        pose_msg.position.y = position[1]
        pose_msg.position.z = position[2]

        pose_msg.orientation.x = orientation[0]
        pose_msg.orientation.y = orientation[1]
        pose_msg.orientation.z = orientation[2]
        pose_msg.orientation.w = orientation[3]

        # print(tf_conversions.transformations.quaternion_matrix(orientation))
        # print(orientation_matrix)
        return pose_msg

    def get_soccer_position_msg(self):
        soccer_position_msg = Point()
        soccer_node = self.robot_node.getFromDef("Soccer")
        position = soccer_node.getPosition()
        soccer_position_msg.x = position[0]
        soccer_position_msg.y = position[1]
        soccer_position_msg.z = position[2]    
        return soccer_position_msg    
        
    def publish_robot_pose(self):
        self.pub_robot_pose_1.publish(self.get_pose_msg_1())
        self.pub_robot_pose_2.publish(self.get_pose_msg_2())
    
    def publish_soccer_pose(self):
        self.pub_soccer_pose.publish(self.get_soccer_position_msg())


r = RobotController()
while not rospy.is_shutdown():
    r.step()
        # print(3)