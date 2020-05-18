#!/usr/bin/env python  
import roslib
import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Pose

def handle_aruco_pose(msg, markerID):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.position.x, msg.position.y, msg.position.z),
    (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w),
    rospy.Time.now(),
    "camera_aruco_ID_%s" % markerID,
    "camera_rgb_optical_frame")



if __name__ == '__main__':
    rospy.init_node('camera_rgb_aruco_tf_broadcaster')
    markerID = [0, 1, 2, 3, 4]
    rospy.Subscriber('/aruco_simple/pose_ID_0', geometry_msgs.msg.Pose, handle_aruco_pose, markerID[0])
    rospy.Subscriber('/aruco_simple/pose_ID_1', geometry_msgs.msg.Pose, handle_aruco_pose, markerID[1])
    #rospy.Subscriber('/aruco_simple/pose_ID_2', geometry_msgs.msg.Pose, handle_aruco_pose, markerID[2])
    #rospy.Subscriber('/aruco_simple/pose_ID_3', geometry_msgs.msg.Pose, handle_aruco_pose, markerID[3])
    #rospy.Subscriber('/aruco_simple/pose_ID_4', geometry_msgs.msg.Pose, handle_aruco_pose, markerID[4])
    rospy.spin()