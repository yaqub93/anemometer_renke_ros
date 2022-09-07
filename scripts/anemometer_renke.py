#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import serial
import numpy as np

def deg2rad(deg):
	return deg/180.0*np.pi 

if __name__ == '__main__':
	rospy.loginfo('Initializing anemometer renke node...')

	ns = rospy.get_namespace()
	rospy.init_node('anemometer_renke', anonymous=True) # initialize rosnode

	port = rospy.get_param('~port','/dev/ttyUSB0')
	baud_rate = rospy.get_param('~baud_rate',4800)

	wind_dir_deg_pub = rospy.Publisher(ns+'wind_direction_deg', Float64, queue_size=10)
	wind_dir_rad_pub = rospy.Publisher(ns+'wind_direction_rad', Float64, queue_size=10)
	wind_speed_pub = rospy.Publisher(ns+'wind_speed', Float64, queue_size=10)

	rate = rospy.Rate(10)

	ser = serial.Serial(port=port,baudrate=baud_rate,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)
	while not rospy.is_shutdown():

		array_req = [0x01,0x03,0x00,0x00,0x00,0x02,0xC4,0x0B]
		ser.write(array_req)

		reply = ser.readlines()
		reply_values = [x for x in bytearray(reply[0])]
		
		wind_speed = Float64()
		wind_speed.data = ((reply_values[3] << 8) + reply_values[4])/100.0
		wind_dir_deg = Float64() 
		wind_dir_deg.data = (reply_values[5] << 8) + reply_values[6]
		wind_dir_rad = Float64() 
		wind_dir_rad.data = deg2rad(wind_dir_deg.data)

		rospy.loginfo('wind speed: '+str(wind_speed.data)+' m/s') 
		rospy.loginfo('wind direction: '+str(wind_dir_deg.data)+' degree') 
		rospy.loginfo('wind direction: '+str(wind_dir_rad.data)+' rad') 

		wind_speed_pub.publish(wind_speed)
		wind_dir_deg_pub.publish(wind_dir_deg)
		wind_dir_rad_pub.publish(wind_dir_rad)

		# give a processing space to callback
		rate.sleep()
