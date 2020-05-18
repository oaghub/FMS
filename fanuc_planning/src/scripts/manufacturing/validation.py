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
from std_msgs.msg import Float32MultiArray, Int8MultiArray, MultiArrayDimension


''' Gathering infomation form the ERP (about the "object" that are requested for manufacturing)'''
def handle_manufacturing_manager_request(msg):
    global requested_object
    global flag_request

    requested_object = msg.data
    #print(requested_object)


''' Obtaining the pettion of validation from the assembly node''' 
def handle_validation_trigger(msg):
    global requested_validation
    global flag_request_validation

    requested_validation = msg.data
    #print(requested_object)
    flag_request_validation = True

''' Gathering information from the arduino's contact sensor'''
def handle_contact_sensor_state(msg):
    global arduino_sensor_state
    #global flag_sensor # Not needed because i want to have as many reading as I could

    arduino_sensor_state = msg.data

    #flag_sensor = True


''' Gathering information about the coordinest of the object ponts's of interest'''
## OBJECT 1
def handle_object_1_coordinates(msg):
    global object_coordinates_1
    global flag_coord_obj_1

    object_coordinates_1 = msg.data

    flag_coord_obj_1 = True
    

def handle_parts_object_1(msg):
    global assemb_parts_obj_1
    global flag_asem_parts_obj_1

    assemb_parts_obj_1 = msg.data
    flag_asem_parts_obj_1 = True

## OBJECT 2
def handle_object_2_coordinates(msg):
    global object_coordinates_2
    global flag_coord_obj_2

    object_coordinates_2 = msg.data

    flag_coord_obj_2 = True
    

def handle_parts_object_2(msg):
    global assemb_parts_obj_2
    global flag_asem_parts_obj_2

    assemb_parts_obj_2 = msg.data
    flag_asem_parts_obj_2 = True







''' Main loop in charge of manufacturing the whole object'''
def validation():
    # Incoming message from the assembly
    global requested_validation
    # Incoming message from the arduino
    global arduino_sensor_state
    # Incomming message from the ERP MANAGER
    global requested_object
    # Bin coordinates
    global bin_location
    # Assembly coordinate
    global object_coordinates_1
    global assemb_parts_obj_1
    global flag_coord_obj_1
    global flag_asem_parts_obj_1

    global object_coordinates_2
    global assemb_parts_obj_2
    global flag_coord_obj_2
    global flag_asem_parts_obj_2




    global flag_bin_location
    global  flag_request_validation

    
    flag_coord_obj_1  = False
    flag_asem_parts_obj_1 = False
    flag_coord_obj_2  = False
    flag_asem_parts_obj_2 = False

    flag_bin_location = False
    flag_request_validation = False


    ''' Default configuration for MoveIt'''
    # Initializing the node representing the robot
    moveit_commander.roscpp_initialize(sys.argv)
    
    rospy.init_node('validation')

    ''' SUBSCRIBERS '''
    # ERP MANAGER NODE
    rospy.Subscriber("frp_manager/request/assembly/trigger", Int8MultiArray, handle_manufacturing_manager_request)
    # ERP DATABASE
    rospy.Subscriber("database/assembly_coordinates/product/1", Float32MultiArray, handle_object_1_coordinates)
    rospy.Subscriber("database/parts/product/1", Int8MultiArray, handle_parts_object_1) # Array of assembly part IDs
    rospy.Subscriber("database/assembly_coordinates/product/2", Float32MultiArray, handle_object_2_coordinates)
    rospy.Subscriber("database/parts/product/2", Int8MultiArray, handle_parts_object_2)
    rospy.Subscriber("database/products/coordinates/object_2", Float32MultiArray, handle_object_2_coordinates)
    

    # ASSEMBLY
    rospy.Subscriber("assembly/request/validation/trigger", Bool, handle_validation_trigger)
    # ARDUINO CONTACT SENSOR
    rospy.Subscriber("arduino/contact/state", Bool, handle_contact_sensor_state)


    ''' PUBLISHERS'''
    pub_arduino =  rospy.Publisher("arduino/action", Int8, queue_size=1)
    #pub_validation = rospy.Publisher("assembly/request/validation/result", Bool, queue_size=1)
    pub_reassemble = rospy.Publisher("validation/result/parts/missing", Int8MultiArray, queue_size=1)
    pub_reassemble_trigger = rospy.Publisher("validation/request/reassembly/trigger", Bool, queue_size=1)

    
    rospy.sleep(3)
    #sub.unregister() # Unsubscribe from the listener

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "fanuc_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    

    missing_parts = Int8MultiArray()
    missing_parts.layout.dim.append(MultiArrayDimension())
    missing_parts.layout.dim.append(MultiArrayDimension())
    missing_parts.layout.dim[0].label = "id"
    missing_parts.layout.dim[1].label = "missing parts"
    missing_parts.layout.dim[0].size = 5
    missing_parts.layout.dim[1].size = 1
    missing_parts.layout.dim[0].stride = 1*5 # Number of elements in the matrix
    missing_parts.layout.dim[1].stride = 1
    missing_parts.layout.data_offset = 0
    missing_parts.data = [0]*5

    # save a few dimensions:
    dstride0 = missing_parts.layout.dim[0].stride
    dstride1 = missing_parts.layout.dim[1].stride
    offset = missing_parts.layout.data_offset


    print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Validation node initialized...                ' + " " +  '\x1b[0m')

    while not rospy.is_shutdown():
        count = 0
        missing = 0
        operations = 0
    
        if flag_request_validation == True and requested_validation == True:
            print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Gathering coordinates of validation...        ' + " " +  '\x1b[0m')
            rospy.sleep(3)
            flag_request_validation = False
            flag_object_coordinates = globals()['flag_coord_obj_%s'%requested_object[1]]
            flag_assembly_parts = globals()['flag_asem_parts_obj_%s'%requested_object[1]]

            if flag_object_coordinates == True:
                print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Preparing to validate product with ID [%d]...  '%requested_object[1] + " " +  '\x1b[0m')
                object_coordinates = numpy.array(globals()['object_coordinates_%s'%requested_object[1]]) # tuple
                required_parts = numpy.array(numpy.array(globals()['assemb_parts_obj_%s'%requested_object[1]]))
                assembly_parts = required_parts[numpy.nonzero(required_parts)] # Selecting and sending only the nonzero valuse
                operations = len(assembly_parts)
                flag_object_coordinates= False
                flag_assembly_parts = False
                #print(assembly_parts)
                #print(operations)

                object_coordinates = object_coordinates[0:operations*7] # Array containing all the object coordinates (one after another)
                #print(object_coordinates)
                assembly_coordinates = numpy.reshape(object_coordinates,(operations,7)) # Reshaping the location of the bins in a matrix of size [operations, 7]
                #print(assembly_coordinates)


                '''STAGE 0 : Change the tool of the gripper'''
                print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Enabling validation tool...                   ' + " " +  '\x1b[0m')
                pub_arduino.publish(1) # 1 = on adn -1 = off
                rospy.sleep(5) # Giving time to the electromagnet to feed the coil


                '''  Loop to select, in order, the desired bin locations corresponding to each one of the coordinates of assembly '''
                while count < operations:
                    print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Validation initialized...                     ' + " " +  '\x1b[0m')
                    print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Operations required = %d                       '%operations + " " +  '\x1b[0m')
                    goal_pose = geometry_msgs.msg.Pose()
                    for i in range(0,operations): ## selecciono el id de la pieza
                        if assembly_parts[i] == 0:
                            pass
                        else:
                            placing_location = assembly_coordinates[i][:]
                            #print(placing_location)
                            #print(count)



                            '''STAGE I : Move to the target position'''
                            print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Moving to part location with ID = [%d]         '%i + " " +  '\x1b[0m')
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
                            
                            
                            '''STAGE II : Assembly the part in its goal location'''
                            print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Validating part with ID [%d]...              '%assembly_parts[i] + " " +  '\x1b[0m')
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
                            print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Checking part with ID [%d]...                '%assembly_parts[i] + " " +  '\x1b[0m')
                            rospy.sleep(5)

                            if arduino_sensor_state != True:
                                print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[7;49;31m'+ " " + 'Part with ID [%d] not detected!              '%assembly_parts[i] + " " +  '\x1b[0m')
                                print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[7;49;31m'+ " " + 'Adding part to assemble queue...              ' + " " +  '\x1b[0m')
                                missing_parts.data[offset + 0 + dstride1*i] = assembly_parts[i]
                                missing += 1
                            else:
                                print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[7;49;92m'+ " " + 'Part with ID [%d] detected!                  '%assembly_parts[i] + " " +  '\x1b[0m')
                            
                            
                            
                            '''STAGE III : Leaving the object'''
                            print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Leaving the object...                         ' + " " +  '\x1b[0m')
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
                            
                            
                            '''STAGE IV : Reseting the position after validation'''
                            move_group.set_named_target("camera_down")
                            move_group.go(wait=True)
                            move_group.stop()
                            
                            
                            ''' STAGE V : Shuting everything down'''     
                            move_group.stop()
                            move_group.clear_pose_targets() 
                            #moveit_commander.roscpp_shutdown()
                            count += 1
                            print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Operation [%d]'%count + ' out of [%d]'%operations + ' completed!           ' " " +  '\x1b[0m')
                            rospy.sleep(2)
                            
                            if count == operations:
                                print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + 'Validation of poduct with ID [%d] finished!    '%requested_object[1] + " " +  '\x1b[0m')
                                rospy.sleep(2)
                                #pub_validation.publish(True)
                                if missing != 0:
                                    print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[7;49;31m'+ " " + 'Some parts have not been detected...          ' + " " +  '\x1b[0m')
                                    print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[7;49;31m'+ " " + 'Requesting assembly corrections...            ' + " " +  '\x1b[0m')
                                    # Publish the missing parts for reassemble
                                    pub_reassemble.publish(missing_parts)
                                    pub_reassemble_trigger.publish(True)
                                    
                                else:
                                    print('\x1b[6;31;43m' + ' (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[7;49;92m'+ " " + 'Product with ID [%d] finished!                 '%requested_object[1] + " " +  '\x1b[0m')     
                                    
                                    ''' Afther the whole operation change the tool position, i.e., switch to the magnet'''
                                    pub_arduino.publish(-1)
                                    rospy.sleep(3) # Giving time the tool to change the effector

            else: 
                pass
        else:
            #print('\x1b[6;31;43m' + '>>> (Fanuc ARC Mate 120iBe)(VALID) >>' + '\x1b[0m' + '\x1b[6;30;43m'+ " " + ' Waiting...  ' + " " +  '\x1b[0m')
            rospy.sleep(2)



if __name__ == '__main__':   
    try:
        validation()
    except rospy.ROSInterruptException:
        pass