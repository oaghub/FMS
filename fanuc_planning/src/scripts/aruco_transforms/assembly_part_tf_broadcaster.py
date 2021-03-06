#!/usr/bin/env python  
import roslib
import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Pose

def handle_aruco_pose(msg, markerID):
    br = tf.TransformBroadcaster()
    # Coordinates of the assembly parts from the center of the aruco marker
    br.sendTransform((0.2, 0, 0.05),
    (0,0,0,1),
    rospy.Time.now(),
    "assembly_part_ID_%s" % markerID,
    "world_aruco_ID_%s" % markerID)


if __name__ == '__main__':
    rospy.init_node('world_aruco_tf_broadcaster')
    markerID = [0, 1, 2, 3, 4]
    rospy.Subscriber('world_to_aruco_ID_0/target_pose', geometry_msgs.msg.Pose, handle_aruco_pose, markerID[0])
    rospy.Subscriber('world_to_aruco_ID_1/target_pose', geometry_msgs.msg.Pose, handle_aruco_pose, markerID[1])
    #rospy.Subscriber('world_to_aruco_ID_2/target_pose', geometry_msgs.msg.Pose, handle_aruco_pose, markerID[2])
    #rospy.Subscriber('world_to_aruco_ID_3/target_pose', geometry_msgs.msg.Pose, handle_aruco_pose, markerID[3])
    #rospy.Subscriber('world_to_aruco_ID_4/target_pose', geometry_msgs.msg.Pose, handle_aruco_pose, markerID[4])-->
    rospy.spin()