#!/usr/bin/env python
__author__ = 'flier'

import rospy
from leap_motion.msg import leap
from leap_motion.msg import leapros
from geometry_msgs.msg import Twist


teleop_topic = '/cmd_vel_mux/input/teleop'


low_speed = -0.5
stop_speed = 0
high_speed = 0.5

low_turn = -0.5
stop_turn = 0
high_turn = 0.5

pitch_low_range = -30
pitch_high_range = 30

roll_low_range = -150
roll_high_range = 150


# Callback of the ROS subscriber, just print the received data.
def callback_ros(data):
    global pub

    msg = leapros()
    msg = data
    
    yaw = msg.ypr.x
    pitch = msg.ypr.y
    roll = msg.ypr.z



    twist = Twist()

    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0

#Just sending Twist message
   
    if(pitch > pitch_low_range and pitch < pitch_low_range + 30):
	    twist.linear.x = high_speed; twist.linear.y = 0; twist.linear.z = 0
	    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            rospy.loginfo("Pitch high")
    
    elif(pitch > pitch_high_range and pitch < pitch_high_range + 30):
	    twist.linear.x = low_speed; twist.linear.y = 0; twist.linear.z = 0
	    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            rospy.loginfo("Pitch low")

    
    if(roll > roll_low_range and roll < roll_low_range + 30):
	    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = high_turn
            rospy.loginfo("Roll left")

    
    elif(roll > roll_high_range and roll < roll_high_range + 30):
	    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
	    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = low_turn
            rospy.loginfo("Roll right")
    
    


    pub.publish(twist)

    #rospy.loginfo(rospy.get_name() + ": Roll %s" % msg.ypr.x)
    #rospy.loginfo("\n")
    rospy.loginfo(rospy.get_name() + ": Pitch %s" % msg.ypr.y)
    #rospy.loginfo("\n")
    #rospy.loginfo(rospy.get_name() + ": Yaw %s" % msg.ypr.z)
    #rospy.loginfo("\n")




# Yes, a listener aka subscriber ;) obviously. Listens to: leapmotion/data
def listener():
    global pub
    rospy.init_node('leap_sub', anonymous=True)
    rospy.Subscriber("leapmotion/data", leapros, callback_ros)
    pub = rospy.Publisher(teleop_topic, Twist, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    listener()
