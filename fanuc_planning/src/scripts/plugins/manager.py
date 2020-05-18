#! /usr/bin/env python
import sys
import copy
import rospy
import time
import numpy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String, Int8, Bool
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray, Int8MultiArray, MultiArrayDimension


''' Gathering inforamtion related to the locations of the bins (where assembly parts are located)'''
def handle_detected_parts(msg):
    global detected_parts
    global flag_parts

    detected_parts = msg.data
    flag_parts = True



''' Gathering infomation form the FRP (about the "object" that are requested for manufacturing) and
    the conditions of manufactrung (the type of parts requiered for the assembly of the specified object)
'''

def handle_manufacturing_request(msg):
    global requested_object
    global manufacturing_conditions
    global flag_frp_request
    global timestamp_frp_request # Request specified in time to avoid overlapping

    requested_object = msg.data[0]
    #print(requested_object)
    manufacturing_conditions = msg.data[1::]
    #print(manufacturing_conditions)


    flag_frp_request = True
    timestamp_frp_request = time.time()




def checker(bins,conditions):
    result = False
    global missing

    detc_bins = numpy.array(bins)
    print(detc_bins)
    manf_conditions = numpy.array(conditions)
    conditions = manf_conditions[numpy.nonzero(manf_conditions)]
    print(manf_conditions)
    comparison = numpy.isin(conditions, detc_bins) # array containing the result after the comparison
    
    possible = all(comparison) # check if all the elements are true and returns true or false


    print(possible)
    return possible






''' Main loop in charge of manufacturing the whole object'''
def manager():

    global detected_parts
    global requested_object
    global timestamp_erp_request
    global flag_frp_request
    global flag_parts

    flag_frp_request = False
    flag_parts = False


    ''' SUBSCRIBERS'''
    rospy.Subscriber("map/detected_parts",  Int8MultiArray, handle_detected_parts)
    rospy.Subscriber("frp/manufacturing/product/info", Int8MultiArray, handle_manufacturing_request)


    ''' PUBLISHERS'''
    pub_scan = rospy.Publisher("frp_manager/request/scan/trigger", Bool, queue_size=1)
    pub_assembly = rospy.Publisher("frp_manager/request/assembly/trigger", Int8MultiArray, queue_size=1)


    
    rospy.sleep(3)

    frp_assmbly_request = Int8MultiArray()
    frp_assmbly_request.layout.dim.append(MultiArrayDimension())
    frp_assmbly_request.layout.dim.append(MultiArrayDimension())
    frp_assmbly_request.layout.dim[0].label = "frp_manager"
    frp_assmbly_request.layout.dim[1].label = "[trigger, object_ID]"
    frp_assmbly_request.layout.dim[0].size = 2
    frp_assmbly_request.layout.dim[1].size = 1
    frp_assmbly_request.layout.dim[0].stride = 1*2 # Number of elements in the matrix
    frp_assmbly_request.layout.dim[1].stride = 1
    frp_assmbly_request.layout.data_offset = 0
    frp_assmbly_request.data = [0]*2

    # save a few dimensions:
    dstride0 = frp_assmbly_request.layout.dim[0].stride
    dstride1 = frp_assmbly_request.layout.dim[1].stride
    offset = frp_assmbly_request.layout.data_offset

    

    print('\x1b[7;49;96m' + '======================================================================' + '\x1b[0m')
    print('\x1b[7;49;96m' + '                  <<< (FRP MANAGER) ' + '\x1b[0m' + '\x1b[7;49;96m'+ " " + 'Initialized >>>                 ' + " " +  '\x1b[0m')
    print('\x1b[7;49;96m' + '======================================================================' + '\x1b[0m')

    while not rospy.is_shutdown(): # and flag_obj_coord == True
        if flag_frp_request == True and flag_parts == True:
            timestamp_request = time.time()
            print('\x1b[7;49;92m' + '[%s]'%time.strftime("%H:%M:%S", time.gmtime()) +' (FRP Manager) >>' + '\x1b[0m' + '\x1b[7;49;92m'+ " " + 'Processing incoming request...           ' + " " +  '\x1b[0m')
            print('\x1b[7;49;34m' + '[%s]'%time.strftime("%H:%M:%S", time.gmtime()) +' (FRP Manager) >>' + '\x1b[0m' + '\x1b[7;49;34m'+ " " + 'Checking manufacturing conditions...     ' + " " +  '\x1b[0m')
            
            is_possible = checker(detected_parts,manufacturing_conditions) # Function to see if the request requirements can be fulfiled with the current setup
            
            if is_possible and (timestamp_frp_request < timestamp_request): # To avoid getting the same request as the previous one
                print('\x1b[7;49;92m' + '[%s]'%time.strftime("%H:%M:%S", time.gmtime()) +' (FRP Manager) >>' + '\x1b[0m' + '\x1b[7;49;92m'+ " " + 'Manufacturing conditions fulfilled...    ' + " " +  '\x1b[0m')
                print('\x1b[7;49;34m' + '[%s]'%time.strftime("%H:%M:%S", time.gmtime()) +' (FRP Manager) >>' + '\x1b[0m' + '\x1b[7;49;34m'+ " " + 'Requesting product with ID [%d]...        '%requested_object+ " " +  '\x1b[0m')
                frp_assmbly_request.data[offset + 0 + dstride1*0] = 1 # Trigger the assembly task
                frp_assmbly_request.data[offset + 0 + dstride1*1] = requested_object # Id of the object to be manufactured
                flag_frp_request = False
                pub_assembly.publish(frp_assmbly_request)
                
                rospy.sleep(1)    
                print('\x1b[7;49;34m' + '[%s]'%time.strftime("%H:%M:%S", time.gmtime()) +' (FRP Manager) >>' + '\x1b[0m' + '\x1b[7;49;34m'+ " " + 'Processing assembly operation...         ' + " " +  '\x1b[0m')            

            else: # I scan the workspace to find the missing parts
                    print('\x1b[7;49;31m' + '[%s]'%time.strftime("%H:%M:%S", time.gmtime()) +' (FRP Manager) >>' + '\x1b[0m' + '\x1b[7;49;31m'+ " " + 'Manufacturing conditions not fulfilled...' + " " +  '\x1b[0m')
                    print('\x1b[7;49;31m' + '[%s]'%time.strftime("%H:%M:%S", time.gmtime()) +' (FRP Manager) >>' + '\x1b[0m' + '\x1b[7;49;31m'+ " " + 'Processing scan request...               ' + " " +  '\x1b[0m')

                    flag_frp_request = False
                    pub_scan.publish(True)
                    rospy.sleep(2)

        else:
            rospy.sleep(2)

        


if __name__ == '__main__':   
    try:
        rospy.init_node('frp_manager')
        manager()
    except rospy.ROSInterruptException:
        pass