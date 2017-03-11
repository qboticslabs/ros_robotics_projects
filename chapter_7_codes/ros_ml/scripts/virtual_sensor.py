#!/usr/bin/env python
__author__ = "Lentin Joseph"

'''
This node will simulate a sensor
which output random values from 
0 to 30,000
'''


import rospy
from std_msgs.msg import Int32
import random


def send_data():

    	rospy.init_node('virtual_sensor', anonymous=True)
	rospy.loginfo("Sending virtual sensor data")
	pub = rospy.Publisher('sensor_read', Int32, queue_size=1)
    	rate = rospy.Rate(10) # 10hz

    	while not rospy.is_shutdown():

        	sensor_reading = random.randint(0,30000)
        	pub.publish(sensor_reading)

        	rate.sleep()

if __name__ == '__main__':
    try:
        send_data()
    except rospy.ROSInterruptException:
        pass
