#! /usr/bin/env python

import rospy
import sys
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy
from math import pi
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int8MultiArray, Bool, Int8
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


## Handling the request from the user or the system itself
def handle_scan_request(msg): # User
    global request
    request = msg.data

def handle_manufacturing_request(msg): # System
    global manufacturing_conditions
    global flag_erp_request

    manufacturing_conditions = numpy.array(msg.data[1::])
    #manufacturing_conditions[manufacturing_conditions > 0] = 1
    flag_erp_request = True


## Handling the (previously defined) assembly part containers
# ArUco with ID 0
def handle_aruco_ID_0_location(msg):
    global aruco_ID_0_position
    global flag_ID_0_position
    aruco_ID_0_position = msg.position
    flag_ID_0_position = True
    #print(aruco_ID_0_position)

def handle_aruco_ID_0_orientation(msg):
    global timestamp_0
    global aruco_ID_0_orientation
    global flag_ID_0_orientation  
    aruco_ID_0_orientation = msg.orientation
    flag_ID_0_orientation = True
    timestamp_0 = time.time()

# ArUco with ID 1
def handle_aruco_ID_1_location(msg):
    global aruco_ID_1_position
    global flag_ID_1_position
    aruco_ID_1_position = msg.position
    flag_ID_1_position = True
    #print(aruco_ID_1_position)

def handle_aruco_ID_1_orientation(msg):
    global timestamp_1
    global aruco_ID_1_orientation
    global flag_ID_1_orientation  
    aruco_ID_1_orientation = msg.orientation
    flag_ID_1_orientation = True
    timestamp_1 = time.time()

def handle_production_request(msg):
    global product

    product = msg.data




def mapping():
    global timestamp_0
    global timestamp_1
    global timestamp_request
    global request
    global flag_ID_0_position
    global flag_ID_0_orientation
    global flag_ID_1_position
    global flag_ID_1_orientation
    
    flag_ID_0_position = False
    flag_ID_0_orientation = False
    flag_ID_1_position = False
    flag_ID_1_orientation = False

    global manufacturing_conditions
    global flag_erp_request
    flag_erp_request = False
    global product

    ''' SUBSCRIBERS '''
    ## Topics to extract the location of the bins
    rospy.Subscriber("world_assembly_part_ID_0/target_pose", Pose, handle_aruco_ID_0_location)
    rospy.Subscriber("world_assembly_part_ID_1/target_pose", Pose, handle_aruco_ID_1_location)

    ## Topics to extract the orientations of the bins
    rospy.Subscriber("/aruco_simple/pose_ID_0", Pose, handle_aruco_ID_0_orientation)
    rospy.Subscriber("/aruco_simple/pose_ID_1", Pose, handle_aruco_ID_1_orientation)


    rospy.Subscriber("scan/request/map/trigger", Bool, handle_scan_request)

    ## Topic to extract the conditions to fulfill the request from the frp
    rospy.Subscriber("frp/manufacturing/product/info", Int8MultiArray, handle_manufacturing_request)

    rospy.Subscriber("user/manufacturing/request", Int8, handle_production_request)

    ''' PUBLISHERS '''
    pub_locations = rospy.Publisher("map/locations", Float32MultiArray, queue_size=1)
    pub_states = rospy.Publisher("map/detected_parts", Int8MultiArray, queue_size=1)
    pub_map_result = rospy.Publisher("map/result", Bool, queue_size=1)

    pub_failed_request = rospy.Publisher("user/manufacturing/request", Int8, queue_size=1)



    # 2D Array that will contain the map in terms of x - y - z  ArUco coordinates 
    aruco_map = Float32MultiArray()
    aruco_map.layout.dim.append(MultiArrayDimension())
    aruco_map.layout.dim.append(MultiArrayDimension())
    aruco_map.layout.dim[0].label = "xyz_xyzw"
    aruco_map.layout.dim[1].label = "aruco id location"
    aruco_map.layout.dim[0].size = 7
    aruco_map.layout.dim[1].size = 5
    aruco_map.layout.dim[0].stride = 7*5 # Number of elements in the matrix
    aruco_map.layout.dim[1].stride = 5
    aruco_map.layout.data_offset = 0
    aruco_map.data = [0]*35

    # save a few dimensions:
    dstride0 = aruco_map.layout.dim[0].stride
    dstride1 = aruco_map.layout.dim[1].stride
    offset = aruco_map.layout.data_offset


    

    bin_state = Int8MultiArray()
    bin_state.layout.dim.append(MultiArrayDimension())
    bin_state.layout.dim.append(MultiArrayDimension())
    bin_state.layout.dim[0].label = "id"
    bin_state.layout.dim[1].label = "detected bin"
    bin_state.layout.dim[0].size = 5
    bin_state.layout.dim[1].size = 1
    bin_state.layout.dim[0].stride = 1*5 # Number of elements in the matrix
    bin_state.layout.dim[1].stride = 1
    bin_state.layout.data_offset = 0
    bin_state.data = [0]*5

    # save a few dimensions:
    dstride0 = bin_state.layout.dim[0].stride
    dstride1 = bin_state.layout.dim[1].stride
    offset = bin_state.layout.data_offset


    rospy.sleep(1)

    tmp_aruco_map = numpy.zeros((5,7)) # 2D array layout
    tmp_bin_state = numpy.zeros((1,5)) # 1D array layot

    scene.remove_world_object()
    p = PoseStamped()

    request = False
    trigger = False

    print('\x1b[7;49;96m' + '=================================' + '\x1b[0m')
    print('\x1b[7;49;96m' + '    <<< (MAP) ' + '\x1b[0m' + '\x1b[7;49;96m'+ " " + 'Initialized >>>  ' + " " +  '\x1b[0m')
    print('\x1b[7;49;96m' + '=================================' + '\x1b[0m')

    while not rospy.is_shutdown():

        if request == True:
            
            if trigger == False:
                aruco_map.data = [0]*35
                bin_state.data = [0]*5
                pub_locations.publish(aruco_map)
                pub_states.publish(bin_state)
                rospy.sleep(2)
                #print("I have just recieved a scan request!")
                timestamp_request = time.time() 
                trigger = True
                ## Only removing the displayed bins
                ''' en lugar de poner todo manualmente que sea todo automatico'''
                scene.remove_world_object("bin_0")
                scene.remove_world_object("part_0")
                scene.remove_world_object("bin_1")
                scene.remove_world_object("part_1")

            


            if flag_ID_0_position == True and flag_ID_0_orientation == True and (timestamp_request < timestamp_0): # Condicion de tiempo para que posicion y orientacion sean differentes
                ''' ArUco ID: 0 (Bin containing assembly parts with ID 0)
                    these coordinates specify the location of the object to be grasp'''
                aruco_map.data[offset + 0 + dstride1*0] = aruco_ID_0_position.x # Filling element [0]
                aruco_map.data[offset + 0 + dstride1*1] = aruco_ID_0_position.y # Filling element [1]
                aruco_map.data[offset + 0 + dstride1*2] = aruco_ID_0_position.z # Filling element [2]
                aruco_map.data[offset + 0 + dstride1*3] = aruco_ID_0_orientation.x # Filling element [3]
                aruco_map.data[offset + 0 + dstride1*4] = aruco_ID_0_orientation.y # Filling element [4]
                aruco_map.data[offset + 0 + dstride1*5] = aruco_ID_0_orientation.z # Filling element [5]
                aruco_map.data[offset + 0 + dstride1*6] = aruco_ID_0_orientation.w # Filling element [6]


                bin_state.data[offset + 0 + dstride1*0] = 100 # id container detected 


                print('\x1b[7;49;34m' + ' (MAP) >>' + '\x1b[0m' + '\x1b[7;49;34m'+ " " + 'Bin with ID 0 detected                                      ' + " " +  '\x1b[0m')


                flag_ID_0_position = False
                flag_ID_0_orientation = False

                
                # Spawning the obstacle from detected aruco frame
                p.header.frame_id = "world_aruco_ID_0" #robot.get_planning_frame()
                p.pose.position.x = 0.15 #locations[0]
                p.pose.position.y = 0 #locations[1]
                p.pose.position.z = 0.025 #locations[2]
                scene.add_box("bin_0", p, (0.3, 0.15, 0.05))

                # Spawning the location of the assebly part as na obstacle
                p.header.frame_id = "assembly_part_ID_0" #robot.get_planning_frame()
                p.pose.position.x = 0 #locations[0]
                p.pose.position.y = 0 #locations[1]
                p.pose.position.z = 0.005 #locations[2]
                scene.add_cylinder("part_0", p, 0.01, 0.02)
                
                pub_locations.publish(aruco_map)
                pub_states.publish(bin_state)

            

            if flag_ID_1_position == True and flag_ID_1_orientation == True and (timestamp_request < timestamp_1): # Condition to have always new readings (when requested)
                ''' ArUco ID: 1 (Bin containing assembly parts with ID 1)
                    these coordinates specify the location of the object to be grasp'''
                aruco_map.data[offset + 0 + dstride1*7] = aruco_ID_1_position.x # Filling element [0]
                aruco_map.data[offset + 0 + dstride1*8] = aruco_ID_1_position.y # Filling element [1]
                aruco_map.data[offset + 0 + dstride1*9] = aruco_ID_1_position.z # Filling element [2]
                aruco_map.data[offset + 0 + dstride1*10] = aruco_ID_1_orientation.x # Filling element [3]
                aruco_map.data[offset + 0 + dstride1*11] = aruco_ID_1_orientation.y # Filling element [4]
                aruco_map.data[offset + 0 + dstride1*12] = aruco_ID_1_orientation.z # Filling element [5]
                aruco_map.data[offset + 0 + dstride1*13] = aruco_ID_1_orientation.w # Filling element [6]


                bin_state.data[offset + 0 + dstride1*1] = 101 #id container detected 



                print('\x1b[7;49;34m' + ' (MAP) >>' + '\x1b[0m' + '\x1b[7;49;34m'+ " " + 'Bin with ID 1 detected                                      ' + " " +  '\x1b[0m')


                flag_ID_1_position = False
                flag_ID_1_orientation = False
 
                
                # Spawning the obstacle from detected aruco frame
                p.header.frame_id = "world_aruco_ID_1" #robot.get_planning_frame()
                p.pose.position.x = 0.15 #locations[0]
                p.pose.position.y = 0 #locations[1]
                p.pose.position.z = 0.025 #locations[2]
                scene.add_box("bin_1", p, (0.3, 0.15, 0.05))

                # Spawning the location of the assebly part as an obstacle
                p.header.frame_id = "assembly_part_ID_1" #robot.get_planning_frame()
                p.pose.position.x = 0 #locations[0]
                p.pose.position.y = 0 #locations[1]
                p.pose.position.z = 0.005 #locations[2]
                scene.add_cylinder("part_1", p, 0.01, 0.02)
                
                pub_locations.publish(aruco_map)
                pub_states.publish(bin_state)

                #rospy.sleep(1)

            detected = numpy.array(bin_state.data)
            comparison = numpy.isin(manufacturing_conditions,detected)
            complete = comparison.all()

            if complete:
                request = False
                trigger = False
                print('\x1b[7;49;92m' + ' (MAP) >>' + '\x1b[0m' + '\x1b[7;49;92m'+ " " + 'All assembly parts containers have been detected!           ' + " " +  '\x1b[0m')
                pub_map_result.publish(False) # Everything has been detected
                pub_failed_request.publish(product)

            
            elif (time.time()) > (timestamp_request + 20.00): # 30 seconds of scanning before timeout
                pub_map_result.publish(False)
                request = False
                trigger = False
                print('\x1b[7;49;31m' + ' (MAP) >>' + '\x1b[0m' + '\x1b[7;49;31m'+ " " + 'After 20.0 seconds the conditions have not been fulfilled...' + " " +  '\x1b[0m')
                print('\x1b[7;49;31m' + ' (MAP) >>' + '\x1b[0m' + '\x1b[7;49;31m'+ " " + 'Shutting down the MAP service...                            ' + " " +  '\x1b[0m')
                print('\x1b[7;49;31m' + ' (MAP) >>' + '\x1b[0m' + '\x1b[7;49;31m'+ " " + 'Please, check the workspace due to missing containers...    ' + " " +  '\x1b[0m')
                rospy.sleep(1)
                ''' automatizad de manera que cuando no sea posible, que se relance el pedido'''
            else:
                pass


        
        else:
            pub_locations.publish(aruco_map)
            pub_states.publish(bin_state)
            rospy.sleep(2)
            # Do nothing until next request

        





if __name__ == '__main__':   
    try:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('map')
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        mapping()
    except rospy.ROSInterruptException:
        pass