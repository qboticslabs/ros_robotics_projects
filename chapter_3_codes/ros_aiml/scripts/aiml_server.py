#!/usr/bin/env python


import rospy
import aiml
import os
import sys


from std_msgs.msg import String

rospy.init_node('aiml_server')
mybot = aiml.Kernel()
response_publisher = rospy.Publisher('response',String,queue_size=10)



def load_aiml(xml_file):

	data_path = rospy.get_param("aiml_path")
	print data_path
	os.chdir(data_path)


	if os.path.isfile("standard.brn"):
		mybot.bootstrap(brainFile = "standard.brn")

	else:
		mybot.bootstrap(learnFiles = xml_file, commands = "load aiml b")
		mybot.saveBrain("standard.brn")


        

def callback(data):

	input = data.data
	response = mybot.respond(input)
	rospy.loginfo("I heard:: %s",data.data)
	rospy.loginfo("I spoke:: %s",response)
	response_publisher.publish(response)
  

def listener():

	rospy.loginfo("Starting ROS AIML Server")
	rospy.Subscriber("chatter", String, callback)
    
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':


  
	load_aiml('startup.xml')
	listener()

