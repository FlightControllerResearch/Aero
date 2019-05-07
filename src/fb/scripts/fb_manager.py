#!/usr/bin/env python2.7
# @brief This node is responsible for sending and reading relevant data from the Google Firebase to interact with the iOS application.
# @author Luke Armbruster
from firebase import firebase
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import NavSatFix
from rtk.msg import HeadingSpeed
import time

ref = firebase.FirebaseApplication('https://sd-d2u.firebaseio.com', None) # public firebase 

def gpsCallback(data):
	ref.put('', 'Current Position', str(data.latitude) + ',' + str(data.longitude) + ',' + str(data.altitude) + ',' + str(data.status.status) + ',' + str(data.status.service))

def controllerCallback(data):
	data_list = data.data.split(',')

def listener():
	rospy.init_node('fb_listener', anonymous=True)
	rospy.Subscriber("rtk_gpgga", NavSatFix, gpsCallback)
	flight_status_pub = rospy.Publisher('flight_requested_status', Bool, queue_size=1)
	prev_flight_requested_status = False
	flight_requested_status = False
	while not rospy.core.is_shutdown():
		rospy.sleep(0.5)
		try:
			flight_requested_status = ref.get('/Flight Requested', None)
		except: 
			rospy.loginfo("Not connected to database")

		if flight_requested_status != prev_flight_requested_status:
			rospy.loginfo("Flight information changed")
			flight_status_msg = Bool()
			flight_status_msg.data = flight_requested_status
			flight_status_pub.publish(flight_status_msg)
			prev_flight_requested_status = flight_requested_status

if __name__ == '__main__':
  listener()

