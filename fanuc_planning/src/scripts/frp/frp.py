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



def handle_production_request(msg):
    global request
    global flag_request

    request = msg.data

    flag_request = True


def handle_available_products(msg):
    global available_products

    available_products = numpy.array(msg.data)


def handle_constrainst_product_1(msg):
    global contrainst_product_1
    global flag_constraints_prod_1

    contrainst_product_1 = numpy.array(msg.data)
    flag_constraints_prod_1 = True


def handle_constrainst_product_2(msg):
    global contrainst_product_2
    global flag_constraints_prod_2

    contrainst_product_2 = numpy.array(msg.data)
    flag_constraints_prod_2 = True




def frp():
    global request
    global flag_request
    global available_products
    global contrainst_product_1
    global contrainst_product_2
    flag_request = False

    ''' SUBSCRIBERS'''
    rospy.Subscriber("user/manufacturing/request", Int8, handle_production_request)
    rospy.Subscriber("database/product/list", Int8MultiArray, handle_available_products)
    rospy.Subscriber("database/parts/product/1", Int8MultiArray, handle_constrainst_product_1)
    rospy.Subscriber("database/parts/product/2", Int8MultiArray, handle_constrainst_product_2)
    ## If we wish to add more products uncomment the lines + callback function
    #rospy.Subscriber("database/parts/product/3", Int8MultiArray, handle_constrainst_product_3)
    #rospy.Subscriber("database/parts/product/4", Int8MultiArray, handle_constrainst_product_4)
    #rospy.Subscriber("database/parts/product/5", Int8MultiArray, handle_constrainst_product_5)

    ''' PUBLISHERS'''
    pub_product_constraints = rospy.Publisher("frp/manufacturing/product/info", Int8MultiArray, queue_size=1)

    

    frp_request = Int8MultiArray()
    frp_request.layout.dim.append(MultiArrayDimension())
    frp_request.layout.dim.append(MultiArrayDimension())
    frp_request.layout.dim[0].label = "product request"
    frp_request.layout.dim[1].label = "[product_Id, bin_0, bin_1, bin_2, bin_3, bin_4]"
    frp_request.layout.dim[0].size = 6
    frp_request.layout.dim[1].size = 1
    frp_request.layout.dim[0].stride = 1*6 # Number of elements in the matrix
    frp_request.layout.dim[1].stride = 1
    frp_request.layout.data_offset = 0
    frp_request.data = [0]*6

    # save a few dimensions:
    dstride0 = frp_request.layout.dim[0].stride
    dstride1 = frp_request.layout.dim[1].stride
    offset = frp_request.layout.data_offset


    rospy.sleep(1)

   
    print('\x1b[7;49;96m' + '======================================================================' + '\x1b[0m')
    print('\x1b[7;49;96m' + '                     <<< (FRP) ' + '\x1b[0m' + '\x1b[7;49;96m'+ " " + 'Initialized >>>                      ' + " " +  '\x1b[0m')
    print('\x1b[7;49;96m' + '======================================================================' + '\x1b[0m')


    while not rospy.is_shutdown():
        ''' Hacer modificacion para que solo funcione con los productos declarados en la base de datos'''
        if flag_request == True:
            print('\x1b[7;49;92m' + '[%s]'%time.strftime("%H:%M:%S", time.gmtime()) +' (ERP) >>' + '\x1b[0m' + '\x1b[7;49;92m'+ " " + 'Processing request...                            ' + " " +  '\x1b[0m')
            flag_request = False
            
            if request in available_products:
                assembly_conditions = globals()['contrainst_product_%s'%request]
                print(assembly_conditions)

                frp_request.data[offset + 0 + dstride1*0] = request # Object ID
                frp_request.data[offset + 0 + dstride1*1] = assembly_conditions[0] # Object manufacturing condition arrembly part type 1
                frp_request.data[offset + 0 + dstride1*2] = assembly_conditions[1] # Object manufacturing condition arrembly part type 2
                frp_request.data[offset + 0 + dstride1*3] = assembly_conditions[2] # Object manufacturing condition arrembly part type 3
                frp_request.data[offset + 0 + dstride1*4] = assembly_conditions[3] # Object manufacturing condition arrembly part type 4
                frp_request.data[offset + 0 + dstride1*5] = assembly_conditions[4] # Object manufacturing condition arrembly part type 5

                flag_request = False

                print('\x1b[7;49;92m' + '[%s]'%time.strftime("%H:%M:%S", time.gmtime()) +' (ERP) >>' + '\x1b[0m' + '\x1b[7;49;92m'+ " " + 'Requested product with ID[%d]...                  '%request + " " +  '\x1b[0m')
                #print(frp_request.data)
                pub_product_constraints.publish(frp_request)
                
                rospy.sleep(5)

            else:
                print('\x1b[7;49;31m' + '[%s]'%time.strftime("%H:%M:%S", time.gmtime()) +' (ERP) >>' + '\x1b[0m' + '\x1b[7;49;31m'+ " " + 'Requested object with ID[%d] not found...         '%request + " " +  '\x1b[0m')
                rospy.sleep(2)
                            
        else:
            rospy.sleep(2)


     

if __name__ == '__main__':   
    try:
        rospy.init_node('frp')
        frp()
    except rospy.ROSInterruptException:
        pass