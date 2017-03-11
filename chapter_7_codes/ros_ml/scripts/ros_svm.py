#!/usr/bin/python
__author__ = "Lentin Joseph"

'''
This node will train an SVM from a CSV file
and classify the values from virtual sensor
'''

import csv
from sklearn import svm
import numpy as np
import time

import rospy
from std_msgs.msg import Int32

obj = 0
pub = 0
#########################################################################################################

class Classify_Data:
	"""
	This class will help to train a set of sensor readings and classify it which are stored in a csv file.
	The class uses sklearn svm to do the training and prediction
	"""

	def __init__(self):
		self.x = []
		self.y = []
		self.result = 0
		self.clf = 0

	def read_file(self,file_name):
		"""
		This function open the csv data file and form a x and y array for training"
		
		"""	
		with open('../data/'+file_name,'rb') as f:
			reader = csv.reader(f)
			for row in reader:
				if(len(row) == 3): 
					a = int(row[0])
					b = int(row[1])
					c = int(row[2])

					self.x.append([a,b])
					self.y.append(c)

		print "Completed reading"	
		return True

	def train(self):
		"""
		This function will train the SVM 
		"""

		self.clf = svm.SVC(kernel='linear', C = 1.0)
		self.clf.fit(self.x,self.y)

	def predict(self,value):
		"""
		This function will predict the result from the input value
		"""
		try:
			result = self.clf.predict([[value,value]])
			return result[0]
		except:
			return 0
			pass
			
	def __del__(self):
		del self.clf


#########################################################################################################
def read_sensor_data(data):

	global obj
	global pub
	input_data = data.data
	
	result = obj.predict(input_data)

	rospy.loginfo("Input = %d \t Prediction = %d",input_data, result)
	pub.publish(result)


def listener():

	global obj
	global pub

	rospy.loginfo("Starting prediction node")

	rospy.init_node('listener', anonymous = True)

	rospy.Subscriber("sensor_read", Int32, read_sensor_data)

	pub = rospy.Publisher('predict', Int32, queue_size=1)


	obj = Classify_Data()

	obj.read_file('pos_readings.csv')

	obj.train()

	rospy.spin()

if __name__ == '__main__':
	listener()


