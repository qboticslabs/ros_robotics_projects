#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

imu_topic = "/imu_data"

teleop_topic = "/cmd_vel_mux/input/teleop"

#Initialization of variables 
prev_yaw = 0
prev_pitch = 0

dy = 0
dp = 0

low_speed = -4
high_speed = 4

low_turn = -2
high_turn = 2

step_size = 0.02

control_speed = 0
control_turn = 0


try:

	imu_topic = rospy.get_param('/imu_topic')
	teleop_topic = rospy.get_param('/teleop_topic')

	low_speed = int(rospy.get_param('/low_speed'))
	high_speed = int(rospy.get_param('/high_speed'))

	low_turn = int(rospy.get_param('/low_turn'))
	high_turn = int(rospy.get_param('/high_turn'))

	step_size = float(rospy.get_param('/step_size'))

	rospy.loginfo("Loaded parameters from config file")

except:
	rospy.logwarn("Unable to retrieve all parameters, loading defaults")
	pass

#Creating publisher for command velocity
pub = rospy.Publisher(teleop_topic, Twist, queue_size=1)

def Send_Twist(dy,dp,pitch):

	"""	
	Function which send the command velocity to robot according to the changes in IMU
	PITCH AND YAW values

	Args:
		dy: Delta yaw
		dp: Delta dp

	Return:
		The function is not returning anything

	"""


	#Declaring as global to access these values in each loop
	global pub
	global control_speed
	global control_turn 

	#Converting into integer
	dy = int(dy)
	dp = int(dp)
	check_pitch = int(pitch)

	#Stopping the robot if pitch is approximately = 0 and dy =0 
	if (check_pitch < 2 and check_pitch > -2 and dy == 0):
		control_speed = 0
		control_turn = 0
	else:

		#Computing the speed and turn from dp and dy, dp and dy can be positive and 			negative
		control_speed = round(control_speed + (step_size * dp),2)
		control_turn = round(control_turn + ( step_size * dy),2)

			
		#Limiting the maximum values of speed
		if (control_speed > high_speed):
			control_speed = high_speed

		elif (control_turn > high_turn):
			control_turn = high_turn


		#Limiting the minimum values of speed
		if (control_speed < low_speed):
			control_speed = low_speed

		elif (control_turn < low_turn):
			control_turn = low_turn

	#Sending the Twist commands		
        twist = Twist()
        twist.linear.x = control_speed; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
        pub.publish(twist)


def Get_RPY(rpy_data):

	global prev_yaw
	global prev_pitch
	global dy,dp

	#Computing dy and dp
	dy = rpy_data.x - prev_yaw
	dp = rpy_data.y - prev_pitch

	#Calling Send_Twist function
	Send_Twist(dy,dp,rpy_data.y)

	#Saving current values as previous values
	prev_yaw = rpy_data.x
	prev_pitch = rpy_data.y



if __name__ == '__main__':
	rospy.loginfo("Starting the Gesture teleop node")
	rospy.init_node('gesture_teleop',anonymous=True)
	rospy.Subscriber(imu_topic,Vector3,Get_RPY)
	rospy.spin()
