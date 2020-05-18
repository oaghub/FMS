#!/usr/bin/env python  
import roslib
import rospy
import tf
import geometry_msgs.msg
from geometry_msgs.msg import Pose

if __name__ == '__main__':
    rospy.init_node('world_assembly_part_ID_0_tf_listener')

    listener = tf.TransformListener()

    # Publishing the transform of the ArUco seen from the magnet aka end-effector   
    # Is not a target_pose (only the position in terms of location is used)
    pub_aruco_world_pose = rospy.Publisher("world_assembly_part_ID_0/target_pose", geometry_msgs.msg.Pose,queue_size=1)


    #rate = rospy.Rate(10.0) # buffers them for up to 10 seconds
    while not rospy.is_shutdown():
        msg = geometry_msgs.msg.Pose()
        try:
            # Create the transform of interest  (base_link == base_link)
            (position,orientation) = listener.lookupTransform("world", "assembly_part_ID_0", rospy.Time(0))
            
            msg.position.x = position[0]
            msg.position.y = position[1]
            msg.position.z = position[2]
            msg.orientation.x = orientation[0]
            msg.orientation.y = orientation[1]
            msg.orientation.z = orientation[2]
            msg.orientation.w = orientation[3]

            pub_aruco_world_pose.publish(msg)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        
        rospy.sleep(1)

        