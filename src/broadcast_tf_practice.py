#!/usr/bin/env python3
import rospy
import tf
from turtlesim.msg import Pose

def poseCallback(msg, turtlename):
    # Create a transform broadcaster
    transform_broadcaster = tf.TransformBroadcaster()
    
    # Convert current yaw angle into quaternion
    rotation_quaternion = tf.transformations.quaternion_from_euler(0,0,msg.theta)
    
    # Translation vector
    translation_vector = (msg.x, msg.y, 0)
    
    # Current time
    current_time = rospy.Time.now()

    # Broadcast the transformation
    transform_broadcaster.sendTransform(translation_vector, rotation_quaternion, current_time, turtlename+"_frame", "world")

if __name__ == "__main__":
    rospy.init_node("tf_broadcaster")
    
    turtlename = rospy.get_param("~turtle")
    
    sub = rospy.Subscriber("/%s/pose"%turtlename, Pose, poseCallback, turtlename)
    
    rospy.spin()