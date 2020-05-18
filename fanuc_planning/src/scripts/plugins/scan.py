#! /usr/bin/env python

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy
from math import pi
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int8MultiArray, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

''' To let the user ask manually for an scan operation'''
def handle_user_request(msg):
    global scan_user_request
    global flag_user_request
    scan_user_request = msg.data 
    #print(scan_user_request)

    if scan_user_request == True: # To avoid the user by mistake sending wrong value adn triggering the scan operation
        flag_user_request = True


def handle_frp_scan_request(msg):
    global scan_frp_manager_request


    scan_frp_manager_request = msg.data 





def handle_map_result(msg):
    global keep_scanning
    keep_scanning = msg.data




def scan():
    global flag_user_request
    global scan_frp_manager_request

    global keep_scanning

    scan_frp_manager_request = False
    flag_user_request = False

    keep_scanning = True

    ''' SUBSCRIBERS '''
    rospy.Subscriber("user/scan/request", Bool, handle_user_request)
    rospy.Subscriber("frp_manager/request/scan/trigger", Bool, handle_frp_scan_request)
    rospy.Subscriber("map/result", Bool, handle_map_result)
    
    ''' PUBLISHERS '''
    pub_map_request = rospy.Publisher("scan/request/map/trigger", Bool, queue_size=1)

    rospy.sleep(1)

    ## Coordinates that limits the "real" workspace (can be modified to adapt the new workspace), values in radians
    joint_mid_sector_goal= [0,0,0,0,-1.57,0]
    joint_left_sector_goal= [-1.57,0,0,0,-1.57,0]
    joint_right_sector_goal= [1.57,0,0,0,-1.57,0]

    

    print('\x1b[7;49;96m' + '=====================================================================' + '\x1b[0m')
    print('\x1b[7;49;96m' + '                     <<< (SCAN) ' + '\x1b[0m' + '\x1b[7;49;96m'+ " " + 'Initialized >>>                    ' + " " +  '\x1b[0m')
    print('\x1b[7;49;96m' + '=====================================================================' + '\x1b[0m')
    
    while not rospy.is_shutdown():

        if flag_user_request == True or scan_frp_manager_request == True:
            print('\x1b[7;49;92m' + ' (SCAN) >>' + '\x1b[0m' + '\x1b[7;49;92m'+ " " + 'Requesting mapping operation...                          ' + " " +  '\x1b[0m')
            pub_map_request.publish(True)
            rospy.sleep(3)

            while keep_scanning: # While I do not detect the desired parts, keep scanning
                print('\x1b[7;49;92m' + ' (SCAN) >>' + '\x1b[0m' + '\x1b[7;49;92m'+ " " + 'Scanning...                                              ' + " " +  '\x1b[0m')

                # Start moving the arm
                move_group.go(joint_mid_sector_goal, wait=True)
                move_group.stop()
                rospy.sleep(1)

                move_group.go(joint_left_sector_goal, wait=True)
                move_group.stop()
                rospy.sleep(1)

                move_group.go(joint_right_sector_goal, wait=True)
                move_group.stop()
                rospy.sleep(1)

            
            flag_user_request = False
            scan_frp_manager_request = False
            move_group.go(joint_mid_sector_goal, wait=True)
            move_group.stop()
            rospy.sleep(1)
            print('\x1b[7;49;92m' + ' (SCAN) >>' + '\x1b[0m' + '\x1b[7;49;92m'+ " " + 'Scan finished!                                           ' + " " +  '\x1b[0m')
            #moveit_commander.roscpp_shutdown()
            
            move_group.set_named_target("camera_down")
            plan_0 = move_group.go(wait=True)
            move_group.clear_pose_targets()
            keep_scanning = True
            
   
        else:
            rospy.sleep(1) # Don nothing until next user request


    

if __name__ == '__main__':   
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("scan", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "fanuc_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        scan()
    except rospy.ROSInterruptException:
        pass