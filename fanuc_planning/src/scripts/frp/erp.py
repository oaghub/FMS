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



''' TO DO!!'''
''' This node is going to simulatate a queue of products that have been requetsted for manufactuing it 
will keep track of the compeleted products, and in the case of requested a product and not been manufactured from some sort of erro, this
node will append that product to a list of pending products for manufacture, and request them at the end of the manufactuing loop to complete the request'''

def erp():
    global request
    global flag_request
    global available_products
    global contrainst_product_1
    global contrainst_product_2
    flag_request = False

    ''' SUBSCRIBERS'''
    rospy.Subscriber("user/manufacturing/request", Int8, handle_production_request)


    ''' PUBLISHERS'''
    pub_product_constraints = rospy.Publisher("frp/manufacturing/product/info", Int8MultiArray, queue_size=1)

    

    erp_request = Int8MultiArray()
    erp_request.layout.dim.append(MultiArrayDimension())
    erp_request.layout.dim.append(MultiArrayDimension())
    erp_request.layout.dim[0].label = "product request"
    erp_request.layout.dim[1].label = "[units, product_Id]"
    erp_request.layout.dim[0].size = 2
    erp_request.layout.dim[1].size = 1
    erp_request.layout.dim[0].stride = 1*2 # Number of elements in the matrix
    erp_request.layout.dim[1].stride = 1
    erp_request.layout.data_offset = 0
    erp_request.data = [0]*2

    # save a few dimensions:
    dstride0 = erp_request.layout.dim[0].stride
    dstride1 = erp_request.layout.dim[1].stride
    offset = erp_request.layout.data_offset


    rospy.sleep(1)

   
    print('\x1b[7;49;96m' + '======================================================================' + '\x1b[0m')
    print('\x1b[7;49;96m' + '                <<< (MANUFACTURING QUEUE) ' + '\x1b[0m' + '\x1b[7;49;96m'+ " " + 'Initialized >>>                 ' + " " +  '\x1b[0m')
    print('\x1b[7;49;96m' + '======================================================================' + '\x1b[0m')


    while not rospy.is_shutdown():
        ''' Hacer modificacion para que solo funcione con los productos declarados en la base de datos'''
        if flag_request == True:
            print('\x1b[7;49;92m' + '[%s]'%time.strftime("%H:%M:%S", time.gmtime()) +' (ERP) >>' + '\x1b[0m' + '\x1b[7;49;92m'+ " " + 'Processing request...                            ' + " " +  '\x1b[0m')
            flag_request = False
            
            if request in available_products:
                assembly_conditions = globals()['contrainst_product_%s'%request]
                #print(numyp.nonzero(assembly_conditions))

                erp_request.data[offset + 0 + dstride1*0] = request # Object ID
                erp_request.data[offset + 0 + dstride1*1] = assembly_conditions[0] # Object manufacturing condition arrembly part type 1
                erp_request.data[offset + 0 + dstride1*2] = assembly_conditions[1] # Object manufacturing condition arrembly part type 2
                erp_request.data[offset + 0 + dstride1*3] = assembly_conditions[2] # Object manufacturing condition arrembly part type 3
                erp_request.data[offset + 0 + dstride1*4] = assembly_conditions[3] # Object manufacturing condition arrembly part type 4
                erp_request.data[offset + 0 + dstride1*5] = assembly_conditions[4] # Object manufacturing condition arrembly part type 5

                flag_request = False

                print('\x1b[7;49;92m' + '[%s]'%time.strftime("%H:%M:%S", time.gmtime()) +' (ERP) >>' + '\x1b[0m' + '\x1b[7;49;92m'+ " " + 'Requested product with ID[%d]...                  '%request + " " +  '\x1b[0m')
                #print(erp_request.data)
                pub_product_constraints.publish(erp_request)
                
                rospy.sleep(5)

            else:
                print('\x1b[7;49;31m' + '[%s]'%time.strftime("%H:%M:%S", time.gmtime()) +' (ERP) >>' + '\x1b[0m' + '\x1b[7;49;31m'+ " " + 'Requested object with ID[%d] not found...         '%request + " " +  '\x1b[0m')
                rospy.sleep(2)
                            
        else:
            rospy.sleep(2)


     

if __name__ == '__main__':   
    try:
        rospy.init_node('erp')
        erp()
    except rospy.ROSInterruptException:
        pass