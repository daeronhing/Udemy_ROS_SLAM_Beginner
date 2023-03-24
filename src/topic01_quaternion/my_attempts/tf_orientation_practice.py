#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

def from_quaternion_to_degrees(quat):
    q = (quat.x, quat.y, quat.z, quat.w)
    rpy_rad = tf.transformations.euler_from_quaternion(q)
    rpy = tuple(math.degrees(x) for x in rpy_rad) 
    return rpy

def odomCallback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    
    quat = msg.pose.pose.orientation
    
    rpy= from_quaternion_to_degrees(quat)
    
    print("\nOdom:")
    print(rpy)
    
def amclCallback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    
    quat = msg.pose.pose.orientation
    rpy= from_quaternion_to_degrees(quat)
    
    print("amcl")
    print(rpy)
    
if __name__ == "__main__":
    rospy.init_node("practice")
    
    odom_subscriber = rospy.Subscriber("/odom", Odometry, odomCallback)
    # amcl_subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amclCallback)
    
    rospy.spin()