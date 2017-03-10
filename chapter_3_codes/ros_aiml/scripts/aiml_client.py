#!/usr/bin/env python


import rospy
from std_msgs.msg import String



pub = rospy.Publisher('chatter', String,queue_size=10)
rospy.init_node('aiml_client')
r = rospy.Rate(1) # 10hz



while not rospy.is_shutdown():

   input = raw_input("\nEnter your text :> ")
   pub.publish(input)
   r.sleep()

   
