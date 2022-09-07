#!/usr/bin/env python

'''
	Author: Yaqub Aris Prabowo
	Email: yaqub.aris@gmail.com
	Institution: Institut Teknologi Bandung
	Description: A python ROS code for acquiring the data from Renke small ultrasonic wind speed and direction transmitter type RS-CFSFX-N01-3 
	Before using, install this following libraries:
		pip install pyserial
		pip install crcmod

'''

import rospy
from std_msgs.msg import Float64
import serial
import crcmod

if __name__ == '__main__':    
    rospy.loginfo('Initializing anemometer renke node...')
    
    ns = rospy.get_namespace()
    rospy.init_node('anemometer_renke', anonymous=True) # initialize rosnode
	
	port = rospy.get_param('~port','ttyUSB0')
	baud_rate = rospy.get_param('~baud_rate',4800)
    
    wind_dir_deg_pub = rospy.Publisher(ns+'wind_direction_deg', Float64, queue_size=10)
    wind_dir_rad_pub = rospy.Publisher(ns+'wind_direction_rad', Float64, queue_size=10)
    wind_speed_pub = rospy.Publisher(ns+'wind_speed', Float64, queue_size=10)
    
    rate = rospy.Rate(100)
    
	crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
	ser = serial.Serial(port=port,baudrate=baud_rate,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)
    ser.open()
    while not rospy.is_shutdown():

		array_req = [0x01,0x03,0x00,0x00,0x00,0x02,0xC4,0x0B]
		ser.write(array_req)

		reply = ser.readlines()
		reply_values = [x for x in reply[0]]

		crc = crc16(bytearray(reply_values[:-2]))
		high_byte = crc >> 8
		low_byte = crc & 0xff
		
		if high_byte == reply_values[-1] and low_byte == reply_values[-2]:
			rospy.loginfo('wind speed:',((reply_values[3] << 8) + reply_values[4])/100.0,'m/s') 
			rospy.loginfo('wind direction:',((reply_values[5] << 8) + reply_values[6]),'degree') 
		else:
			rospy.logwarn('CRC error')

        # give a processing space to callback
        rate.sleep()
