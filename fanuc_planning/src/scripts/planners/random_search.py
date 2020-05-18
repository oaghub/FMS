#! /usr/bin/env python
import sys
import copy
import rospy
import random
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from math import pi
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint




def handle_map_locations(msg):
    global locations
    global tmp_aruco_map
    global flag

    tmp_aruco_map = np.zeros((5,3)) # 2D array layout
    locations = msg.data
    #print locations[0] # x coordinate ArUco (ID O)
    #print locations[5] # y coordinate ArUco (ID 0)
    #print locations[10] # z coordinate ArUco (ID 0)
    flag = True 



def random_search():
    global locations
    global target
    global flag

    ''' Default configuration for MoveIt'''
    # Initializing the node representing the robot
    moveit_commander.roscpp_initialize(sys.argv)

    flag = False
    
    rospy.init_node("random_search")
    rospy.Subscriber("aruco_map/locations",  Float32MultiArray, handle_map_locations)
    rospy.sleep(2)

    # Provides information such as the robot's kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    # Provides a remote interface for getting, setting, and updating the robot's internal understanding of the
    # surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    # Is an interface to a planning group (group of joints).Used to plan and execute motions
    group_name = "fanuc_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    for i in range(0,1):
        if flag == True:
            #val = random.randint(0,3)        #print("Going to ArUco ID: %d"%randint(0,1))
            #print val
#
            target = geometry_msgs.msg.Pose()

            target.position.x = locations[0+i] -0.01 # Coordinate x of aruco maker ID 0
            target.position.y = locations[5+i] +0.01# Coordinate y of aruco maker ID 0 
            target.position.z = locations[10+i] +0.08 # Coordinate z of aruco maker ID 0
            print(target.position.x)
            print(target.position.y)
            print(target.position.z)

            target.orientation.x = 0.7071 # target_orientation.x
            target.orientation.y = 0.7071 # target_orientation.y
            target.orientation.z = 0 # target_orientation.z
            target.orientation.w = 0 # target_orientation.w

            #print target


            move_group.set_pose_target(target)
            print('\x1b[6;31;43m' + '>>> Fanuc ARC Mate 120iBe >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Approaching ArUco ID:  %d  '%i + " " +  '\x1b[0m')
            plan = move_group.go(wait=True)
            flag = False



    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    plan2 = move_group.go(joint_goal, wait=True)

    ''' STAGE X : Shuting everything down'''
    print('\x1b[6;31;43m' + '>>> Fanuc ARC Mate 120iBe >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Shuting Down...' + " " +  '\x1b[0m')
    move_group.stop()
    flag = False
    move_group.clear_pose_targets() 
    moveit_commander.roscpp_shutdown()
    exit()




if __name__ == '__main__':   
    try:
        random_search()
    except rospy.ROSInterruptException:
        pass