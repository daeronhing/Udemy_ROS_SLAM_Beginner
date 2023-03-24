#!/usr/bin/env python3

import tf
import math
import rospy
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    # init node
    rospy.init_node("follower")
    
    # create a new transform listener
    transform_listener = tf.TransformListener()
    
    # create a second turtle by calling the service
    rospy.wait_for_service("spawn")
    spawner = rospy.ServiceProxy("spawn", Spawn)
    spawner(4,2,0,'turtle2')
    
    turtle_follower_publisher = rospy.Publisher("turtle2/cmd_vel", Twist, queue_size=1)
    
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        try:
            (translation, rotation) = transform_listener.lookupTransform("turtle2/frame", "turtle1/frame", rospy.Time(0))
        except:
            continue
        
        x_follower = translation[0]
        y_follower = translation[1]
        
        angular = 4 * math.atan2(y_follower, x_follower)
        linear = 0.5 * math.sqrt(x_follower ** 2 + y_follower ** 2)
 
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_follower_publisher.publish(cmd)
        
        rate.sleep()       