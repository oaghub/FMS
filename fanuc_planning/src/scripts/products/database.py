#! /usr/bin/env python

import rospy
import sys
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from math import pi
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int8MultiArray, Bool
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

def database():

    ''' PUBLISHERS'''
    pub_product_ids = rospy.Publisher("database/product/list", Int8MultiArray, queue_size=1)
    pub_product_1 = rospy.Publisher("database/assembly_coordinates/product/1", Float32MultiArray, queue_size=1)
    pub_parts_1 = rospy.Publisher("database/parts/product/1", Int8MultiArray, queue_size=1)
    pub_product_2 = rospy.Publisher("database/assembly_coordinates/product/2", Float32MultiArray, queue_size=1)
    pub_parts_2 = rospy.Publisher("database/parts/product/2", Int8MultiArray, queue_size=1)


    # 2D Array that will contain the assembly coordinates + orientation
    coord_obj = Float32MultiArray()
    coord_obj.layout.dim.append(MultiArrayDimension())
    coord_obj.layout.dim.append(MultiArrayDimension())
    coord_obj.layout.dim[0].label = "[position, orientation]"
    coord_obj.layout.dim[0].size = 7# Elements in a row
    coord_obj.layout.dim[0].stride = 5*7 # Number of elements in the matrix (5 coordinates)
    coord_obj.layout.dim[1].label = "coordinates"
    coord_obj.layout.dim[1].size = 7 # number of columns
    coord_obj.layout.dim[1].stride = 1 # Just one matrix
    coord_obj.layout.data_offset = 0
    coord_obj.data = [0]*35

    # save a few dimensions:
    dstride0 = coord_obj.layout.dim[0].stride
    dstride1 = coord_obj.layout.dim[1].stride
    offset = coord_obj.layout.data_offset

    assemb_parts = Int8MultiArray()
    assemb_parts.layout.dim.append(MultiArrayDimension())
    assemb_parts.layout.dim.append(MultiArrayDimension())
    assemb_parts.layout.dim[0].label = "[part_IDs]"
    assemb_parts.layout.dim[0].size = 5# Elements in a row
    assemb_parts.layout.dim[0].stride = 5*1 # Number of elements in the matrix (5 coordinates)
    assemb_parts.layout.dim[1].label = "required parts"
    assemb_parts.layout.dim[1].size = 5 # number of columns
    assemb_parts.layout.dim[1].stride = 1 # Just one matrix
    assemb_parts.layout.data_offset = 0
    assemb_parts.data = [0]*5

    # save a few dimensions:
    dstride0 = assemb_parts.layout.dim[0].stride
    dstride1 = assemb_parts.layout.dim[1].stride
    offset = assemb_parts.layout.data_offset

    print('\x1b[7;49;94m' + '===========================================' + '\x1b[0m')
    print('\x1b[7;49;94m' + '    <<< (FRP DATABASE) ' + '\x1b[0m' + '\x1b[7;49;94m'+ " " + 'Initialized >>>   ' + " " +  '\x1b[0m')
    print('\x1b[7;49;94m' + '===========================================' + '\x1b[0m')
    print('\x1b[7;49;94m' + ' Available products to manufacture:        ' + '\x1b[0m')
    print('\x1b[7;49;94m' + '   * Product with ID: 1                    ' + '\x1b[0m')
    print('\x1b[7;49;94m' + '   * Product with ID: 2                    ' + '\x1b[0m')
    print('\x1b[7;49;94m' + '                                           ' + '\x1b[0m')
  
    rospy.sleep(1)


    while not rospy.is_shutdown():
        ''' PRODUCT WITH ID 1 '''
        coord_obj_1 = coord_obj
        assemb_parts_1 = assemb_parts

        # Coordinates of assembly
        coord_obj_1.data[offset + dstride0*0 + dstride1*0] = 1.20   # Filling element [0] position
        coord_obj_1.data[offset + dstride0*0 + dstride1*1] = 0.5   # Filling element [1] position
        coord_obj_1.data[offset + dstride0*0 + dstride1*2] = 0.60   # Filling element [2] postition
        coord_obj_1.data[offset + dstride0*0 + dstride1*3] = 0.7071 # Filling element [3] orientation
        coord_obj_1.data[offset + dstride0*0 + dstride1*4] = 0.7071 # Filling element [4] orientation
        coord_obj_1.data[offset + dstride0*0 + dstride1*5] = 0.0    # Filling element [5] orientation
        coord_obj_1.data[offset + dstride0*0 + dstride1*6] = 0.0    # Filling element [6] orientation

        coord_obj_1.data[offset + dstride0*0 + dstride1*7] = 1
        coord_obj_1.data[offset + dstride0*0 + dstride1*8] = -0.5
        coord_obj_1.data[offset + dstride0*0 + dstride1*9] = 0.60
        coord_obj_1.data[offset + dstride0*0 + dstride1*10] = 0.7071
        coord_obj_1.data[offset + dstride0*0 + dstride1*11] = 0.7071
        coord_obj_1.data[offset + dstride0*0 + dstride1*12] = 0.0
        coord_obj_1.data[offset + dstride0*0 + dstride1*13] = 0.0

        # Required assembly parts
        assemb_parts_1.data[offset + dstride0*0 + dstride1*0] = 100 # Filling element [0] ID of the requiered part for the first location
        assemb_parts_1.data[offset + dstride0*0 + dstride1*1] = 101 # Filling element [1] ''  ''
        assemb_parts_1.data[offset + dstride0*0 + dstride1*2] = 0 # Filling element [2] ''  ''
        assemb_parts_1.data[offset + dstride0*0 + dstride1*3] = 0 # Filling element [3] ''  ''


        pub_parts_1.publish(assemb_parts_1)
        pub_product_1.publish(coord_obj_1)
        rospy.sleep(1)
        


        
        ''' PRODUCT WITH ID 2 '''

        coord_obj_2 = coord_obj
        assemb_parts_2 = assemb_parts

        # Coordinates of assembly
        coord_obj_2.data[offset + dstride0*0 + dstride1*0] = 1   # Filling element [0] position
        coord_obj_2.data[offset + dstride0*0 + dstride1*1] = -0.5   # Filling element [1] position
        coord_obj_2.data[offset + dstride0*0 + dstride1*2] = 0.60   # Filling element [2] postition
        coord_obj_2.data[offset + dstride0*0 + dstride1*3] = 0.7071 # Filling element [3] orientation
        coord_obj_2.data[offset + dstride0*0 + dstride1*4] = 0.7071 # Filling element [4] orientation
        coord_obj_2.data[offset + dstride0*0 + dstride1*5] = 0.0    # Filling element [5] orientation
        coord_obj_2.data[offset + dstride0*0 + dstride1*6] = 0.0    # Filling element [6] orientation

        coord_obj_1.data[offset + dstride0*0 + dstride1*7] = 0.0
        coord_obj_1.data[offset + dstride0*0 + dstride1*8] = 0.0
        coord_obj_1.data[offset + dstride0*0 + dstride1*9] = 0.0
        coord_obj_1.data[offset + dstride0*0 + dstride1*10] = 0.0
        coord_obj_1.data[offset + dstride0*0 + dstride1*11] = 0.0
        coord_obj_1.data[offset + dstride0*0 + dstride1*12] = 0.0
        coord_obj_1.data[offset + dstride0*0 + dstride1*13] = 0.0

        # Required assembly parts
        assemb_parts_2.data[offset + dstride0*0 + dstride1*0] = 101 # Filling element [0] ID of the requiered part for the first location
        assemb_parts_2.data[offset + dstride0*0 + dstride1*1] = 0 # Filling element [1] ''  ''
        assemb_parts_2.data[offset + dstride0*0 + dstride1*2] = 0 # Filling element [2] ''  ''
        assemb_parts_2.data[offset + dstride0*0 + dstride1*3] = 0 # Filling element [3] ''  ''

        
        pub_parts_2.publish(assemb_parts_2)
        pub_product_2.publish(coord_obj_2)



        available_products = assemb_parts
        available_products.data[offset + dstride0*0 + dstride1*0] = 1 # Filling element [0] ID of the requiered part for the first location
        available_products.data[offset + dstride0*0 + dstride1*1] = 2 # Filling element [1] ''  ''
        available_products.data[offset + dstride0*0 + dstride1*2] = 99 # Filling element [2] ''  ''
        available_products.data[offset + dstride0*0 + dstride1*3] = 99 # Filling element [3] ''  ''
        
        pub_product_ids.publish(available_products)
        


        rospy.sleep(1)

    
if __name__ == '__main__':   
    try:
        rospy.init_node('database')
        database()
    except rospy.ROSInterruptException:
        pass