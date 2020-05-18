#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy
from math import pi
from std_msgs.msg import String, Int8, Bool
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray, Int8MultiArray

''' Gathering the request for the reassembly process'''
def handle_reassembly_request(msg):
    global reassembly_request
    global flag_reassembly_request

    reassembly_request = msg.data

    flag_reassembly_request = True

''' Gathering infomation form the ERP (about the "object" that are requested for manufacturing)'''
def handle_manufacturing_manager_request(msg):
    global requested_object
    global flag_request

    requested_object = msg.data
    #print(requested_object)
    flag_request = True



''' Gathering inforamtion related to the locations of the bins (where assembly parts are located)'''
def handle_aruco_bin_locations(msg):
    global bin_location
    global flag_bin_location
    
    bin_location = msg.data

    flag_bin_location = True 



''' Gathering information about the coordinest of the object ponts's of interest'''
## OBJECT 1
def handle_object_1_coordinates(msg):
    global object_coordinates_1
    global flag_coord_obj_1

    object_coordinates_1 = msg.data

    flag_coord_obj_1 = True

## OBJECT 2
def handle_object_2_coordinates(msg):
    global object_coordinates_2
    global flag_coord_obj_2

    object_coordinates_2 = msg.data

    flag_coord_obj_2 = True


''' To request a validation after the reassembly process'''
def handle_missing_parts(msg):
    global assemb_missing_parts
    global flag_missing_parts

    assemb_missing_parts = msg.data
    flag_missing_parts = True



    

def assembly():
    # Incomming message from the ERP MANAGER
    global requested_object
    global flag_request
    # Bin coordinates
    global bin_location
    global flag_bin_location
    # Assembly coordinates
    global object_coordinates_1 
    global flag_coord_obj_1
    global object_coordinates_2 
    global flag_coord_obj_2

    # Reassembly request
    global assemb_missing_parts
    global reassembly_request
    global flag_reassembly_request
    global flag_missing_parts


    
    
    
    flag_coord_obj_1  = False
    flag_coord_obj_2  = False
    flag_missing_parts = False
    flag_reassembly_request = False
    flag_request = False
    flag_bin_location = False




    ''' SUBSCRIBERS '''
    # ARUCO MAPPING NODE
    rospy.Subscriber("map/locations", Float32MultiArray, handle_aruco_bin_locations)
    # ERP MANAGER NODE
    rospy.Subscriber("frp_manager/request/assembly/trigger", Int8MultiArray, handle_manufacturing_manager_request)
    # ERP DATABASE
    rospy.Subscriber("database/assembly_coordinates/product/1", Float32MultiArray, handle_object_1_coordinates)
    rospy.Subscriber("database/assembly_coordinates/product/2", Float32MultiArray, handle_object_2_coordinates)

    # VALIDATION NODE (missing parts)
    rospy.Subscriber("validation/result/parts/missing", Int8MultiArray, handle_missing_parts)
    rospy.Subscriber("validation/request/reassembly/trigger", Bool, handle_reassembly_request)



    ''' PUBLISHERS'''
    pub_arduino =  rospy.Publisher("arduino/action", Int8, queue_size=1)
    pub_validation = rospy.Publisher("assembly/request/validation/trigger", Bool, queue_size=1)

    
    rospy.sleep(3)
    #sub.unregister() # Unsubscribe from the listener

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "fanuc_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Reassembly node initialized...                ' + " " +  '\x1b[0m')

    while not rospy.is_shutdown():
        count = 0
        operations = 0
    
        if flag_reassembly_request == True and requested_object[0] == True: # requested_object[0] / fisrt element corresponds to the assembly trigger, if True continue
            print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Gathering coordinates of assembly...          ' + " " +  '\x1b[0m')
            rospy.sleep(3)
            flag_reassembly_request = False
            flag_object_coordinates = globals()['flag_coord_obj_%s'%requested_object[1]]

            if flag_object_coordinates == True:
                print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Preparing to reassemble product with ID [%d]...'%requested_object[1] + " " +  '\x1b[0m')
                object_coordinates = numpy.array(globals()['object_coordinates_%s'%requested_object[1]]) 
                missing_parts = numpy.array(assemb_missing_parts)
                operations = len(missing_parts)
                flag_coord_obj_1 = False
                #print(assemb_missing_parts)
                #print(operations)

                object_coordinates = object_coordinates[0:operations*7] 
                #print(object_coordinates)
                assembly_coordinates = numpy.reshape(object_coordinates,(operations,7))
                #print(assembly_coordinates)
            

            
            

                '''  Loop to select, in order, the desired bin locations corresponding to each one of the coordinates of assembly '''
                while count < operations:
                    print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Reassembly initialized...                     ' + " " +  '\x1b[0m')
                    #print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Operations required = [%d]                    '%operations + " " +  '\x1b[0m')
                    goal_pose = geometry_msgs.msg.Pose()

                    for i in range(0,operations): ## selecciono el id de la pieza
                        if assemb_missing_parts[i] == 100:
                            picking_location = bin_location[0:7] # location of the bin with id 0 (fixed) bin id 0 corresponds with the frist 7 values
                        elif assemb_missing_parts[i] == 101:
                            picking_location = bin_location[7:14]
                        else:
                            count +=1
                            #print('When 0, count = %d'%count)
                            continue

                        placing_location = assembly_coordinates[i][:]
                        #print(placing_location)
                        #print(count)



                        '''STAGE I : Approaching the desired bin '''
                        print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Fetching assembly part with id = %d          '%assemb_missing_parts[i] + " " +  '\x1b[0m')
                        goal_pose.position.x = picking_location[0]
                        goal_pose.position.y = picking_location[1] 
                        goal_pose.position.z = picking_location[2] + 0.30
                        goal_pose.orientation.x = picking_location[3] # 0.7071
                        goal_pose.orientation.y = picking_location[4] # 0.7071
                        goal_pose.orientation.z = picking_location[5] # 0
                        goal_pose.orientation.w = picking_location[6] # 0
                        #print(picking_location)
                        move_group.set_pose_target(goal_pose)
                        plan = move_group.go(wait=True)
                        move_group.stop()
                        #rospy.sleep(5)



                        '''STAGE II : Approach the assembly part to its location'''
                        print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Approaching the part...                       ' + " " +  '\x1b[0m')
                        goal_pose.position.x = picking_location[0]
                        goal_pose.position.y = picking_location[1] 
                        goal_pose.position.z = picking_location[2] + 0.12
                        goal_pose.orientation.x = picking_location[3] # 0.7071
                        goal_pose.orientation.y = picking_location[4] # 0.7071
                        goal_pose.orientation.z = picking_location[5] # 0
                        goal_pose.orientation.w = picking_location[6] # 0
                        move_group.set_pose_target(goal_pose)
                        move_group.go(wait=True)
                        move_group.stop()
                        #rospy.sleep(5)


                        '''STAGE III : Enable the magnet'''
                        print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Enabling electromagnet...                     ' + " " +  '\x1b[0m')
                        pub_arduino.publish(2) # 2 = on adn -2 = off
                        rospy.sleep(5) # Giving time to the electromagnet to feed the coil


                        #
                        ## At this point the assembly part is already grasp
                        #


                        '''STAGE IV : Extract the assembly part from the container'''
                        print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Extracting the part from the container...     ' + " " +  '\x1b[0m')
                        goal_pose.position.z = picking_location[2] + 0.3
                        move_group.set_pose_target(goal_pose)
                        move_group.go(wait=True)
                        move_group.stop()
                        #rospy.sleep(5)


                        #
                        ## At this point the assembly part is already extracted from its conatiener and ready to be assembled into the main object
                        #


                        '''STAGE V : Move to the target position'''
                        print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Moving object to location with ID = [%d]       '%i + " " +  '\x1b[0m')
                        goal_pose.position.x = placing_location[0] 
                        goal_pose.position.y = placing_location[1] 
                        goal_pose.position.z = placing_location[2] + 0.3
                        goal_pose.orientation.x = placing_location[3] # 0.7071
                        goal_pose.orientation.y = placing_location[4] # 0.7071
                        goal_pose.orientation.z = placing_location[5] # 0
                        goal_pose.orientation.w = placing_location[6] # 0
                        move_group.set_pose_target(goal_pose)
                        move_group.go(wait=True)
                        move_group.stop()
                        #rospy.sleep(5)



                        '''STAGE VI : Assembly the part in its goal location'''
                        print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Assemblying part...                           ' + " " +  '\x1b[0m')
                        goal_pose.position.x = placing_location[0] 
                        goal_pose.position.y = placing_location[1]
                        goal_pose.position.z = placing_location[2]
                        goal_pose.orientation.x = placing_location[3] # 0.7071
                        goal_pose.orientation.y = placing_location[4] # 0.7071
                        goal_pose.orientation.z = placing_location[5] # 0
                        goal_pose.orientation.w = placing_location[6] # 0
                        move_group.set_pose_target(goal_pose)
                        move_group.go(wait=True)
                        move_group.stop()
                        #rospy.sleep(5)



                        '''STAGE VII : Disable the magnet'''
                        print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Disabling electromagnet...                    ' + " " +  '\x1b[0m')
                        pub_arduino.publish(-2) # 2 = on adn -2 = off
                        rospy.sleep(5) # Giving time to the electromagnet to feed the coil


                        #
                        ## At this point the part has been assembled to the main object and released from the magnet
                        #


                        '''STAGE VIII : Leaving the object'''
                        print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Leaving the object...                         ' + " " +  '\x1b[0m')
                        goal_pose.position.x = placing_location[0] 
                        goal_pose.position.y = placing_location[1]
                        goal_pose.position.z = placing_location[2] + 0.3
                        goal_pose.orientation.x = placing_location[3] # 0.7071
                        goal_pose.orientation.y = placing_location[4] # 0.7071
                        goal_pose.orientation.z = placing_location[5] # 0
                        goal_pose.orientation.w = placing_location[6] # 0
                        move_group.set_pose_target(goal_pose)
                        move_group.go(wait=True)
                        move_group.stop()
                        #rospy.sleep(5)



                        '''STAGE IX : Reseting the position after assembly'''
                        move_group.set_named_target("camera_down")
                        move_group.go(wait=True)
                        move_group.stop()

                        ''' STAGE X : Shuting everything down'''     
                        move_group.stop()
                        #flag = False
                        #flag_orientation = False
                        move_group.clear_pose_targets() 
                        #moveit_commander.roscpp_shutdown()

                        count += 1
                        #print('When not 0, count = %d'%count)
                        #print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Operation [%d]'%count + ' out of [%d]'%operations + ' completed!       ' " " +  '\x1b[0m')
                        rospy.sleep(2)


                #if count == operations:
                print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Finished product with ID = [%d]                '%requested_object[1] + " " +  '\x1b[0m')
                rospy.sleep(2)
                print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(RSSMB) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Proceeding to its validation...               ' + " " +  '\x1b[0m')
                pub_validation.publish(True)  

            else:
                pass      
                
        else:
            rospy.sleep(2)



if __name__ == '__main__':   
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('reassembly')
        assembly()
    except rospy.ROSInterruptException:
        pass