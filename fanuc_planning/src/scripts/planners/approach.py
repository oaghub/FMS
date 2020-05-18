#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray


def handle_aruco_target_location(msg):

    global target_location
    global flag 
    
    target_location = msg.data
    #print(target_location)
    flag = True 


    

def aruco_approach():
    global target_location
    global flag
    flag = False


    ''' Default configuration for MoveIt'''
    # Initializing the node representing the robot
    moveit_commander.roscpp_initialize(sys.argv)
    
    rospy.init_node('fanuc_move_group', anonymous=True)
    sub = rospy.Subscriber("aruco_mapping/locations", Float32MultiArray, handle_aruco_target_location)
    print('\x1b[6;31;43m' + '>>> Fanuc ARC Mate 120iBe >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + ' Gathering ArUco pose...  ' + " " +  '\x1b[0m')

    rospy.sleep(3)
    sub.unregister() # Unsubscribe from the listener

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "fanuc_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

     
    if flag == True:
        ''' Reading the desired aruco pose from  TF Listeners '''

        goal_pose = geometry_msgs.msg.Pose()
        goal_pose.position.x = target_location[0]
        goal_pose.position.y = target_location[1] 
        goal_pose.position.z = target_location[2] + 0.2
       

        goal_pose.orientation.x = target_location[3] # 0.7071
        goal_pose.orientation.y = target_location[4] # 0.7071
        goal_pose.orientation.z = target_location[5] # 0
        goal_pose.orientation.w = target_location[6] # 0
        print(target_location)

        move_group.set_pose_target(goal_pose)
        print('\x1b[6;31;43m' + '>>> Fanuc ARC Mate 120iBe >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Approaching ArUco ID:  0  ' + " " +  '\x1b[0m')
        plan = move_group.go(wait=True)
        move_group.stop()
        
        
        
        
        ''' STAGE X : Shuting everything down'''
        print('\x1b[6;31;43m' + '>>> Fanuc ARC Mate 120iBe >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Shuting Down...' + " " +  '\x1b[0m')
        move_group.stop()
        flag = False
        flag_orientation = False
        move_group.clear_pose_targets() 
        moveit_commander.roscpp_shutdown()

    else:
        print('\x1b[6;31;43m' + '>>> Fanuc ARC Mate 120iBe >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + ' Target not found  ' + " " +  '\x1b[0m')
        pass




if __name__ == '__main__':   
    try:
        aruco_approach()
    except rospy.ROSInterruptException:
        pass