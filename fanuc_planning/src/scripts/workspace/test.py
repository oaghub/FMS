#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose, PoseStamped



def checker(bins,conditions):
    result = False
    detc_bins = numpy.array(bins)
    manf_conditions = numpy.array(conditions)
    available = sum(manf_conditions)
    i = 0
    for cond in manf_conditions:
        if cond == detc_bins[i] and cond == True and detc_bins[i] == True:
            available -= 1
        else:
            pass

        i += 1
        
    if available == 0:
        return True 
    else:
        return False


print(checker([1,0,0],[1,0,1]))