#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Importing packages
import rospy
import serial
import numpy as np
from math import pi
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from imu_driver.msg import imu_msg

def eulerToQuaternion(yaw,pitch,roll):
	yaw = yaw*(pi/180)
	pitch = pitch*(pi/180)
	roll = roll*(pi/180)
	qx = np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2)-np.cos(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
	qy = np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2)+np.sin(roll/2)*np.cos(pitch/2)*np.sin(yaw/2)
	qz = np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2)-np.sin(roll/2)*np.sin(pitch/2)*np.cos(yaw/2)
	qw = np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2)+np.sin(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)
	return [qx,qy,qz,qw]

if __name__ == '__main__':
	# Initializing node
	rospy.init_node('IMU', anonymous=True)

	# Setting the serial port
	serial_port = rospy.get_param('~port','/dev/ttyUSB0')
	serial_baud = rospy.get_param('~baudrate',115200)
	sampling_rate = rospy.get_param('~sampling_rate',5.0)
	port = serial.Serial(serial_port, serial_baud, timeout=3.)

	# Logging for debug 
	rospy.logdebug("Using IMU sensor on port /dev/ttyUSB0 at 115200 baudrate")
	rospy.logdebug("Initializing sensor\\r\\n ...")

	# Creating a publisher object
	imu_pub = rospy.Publisher('imu', imu_msg, queue_size=10)

	rospy.logdebug("Initialization complete")
	
	# Creating a message object
	msg = imu_msg()

	port.write(b"$VNWRG,07,40*XX")
	# freq=port.read(b"$VNRRG,07*XX")
	# print(freq)
	try:
		while not rospy.is_shutdown():
			line = port.readline()
			if line == '':
				rospy.logwarn("IMU: No data")
			else:
				#print(line)
				line_str = line.decode('UTF-8')
				#print(line_str)
				line_arr = line_str.split(',')
				#print(line_arr)
				if '$VNYMR' in line_arr[0]:
					now = rospy.Time.now()
					msg.Header.stamp.secs = now.secs
					msg.Header.stamp.nsecs = now.nsecs
					msg.Header.frame_id = "IMU1_Frame"
					[QX, QY, QZ, QW] = eulerToQuaternion(float(line_arr[1]),float(line_arr[2]),float(line_arr[3]))
					msg.IMU.orientation.x = QX
					msg.IMU.orientation.y = QY
					msg.IMU.orientation.z = QZ
					msg.IMU.orientation.w = QW
					msg.IMU.angular_velocity.x = float(line_arr[10])
					msg.IMU.angular_velocity.y = float(line_arr[11])
					angVelZ = line_arr[12].split('*')
					msg.IMU.angular_velocity.z = float(angVelZ[0])
					msg.IMU.linear_acceleration.x = float(line_arr[7])
					msg.IMU.linear_acceleration.y = float(line_arr[8])
					msg.IMU.linear_acceleration.z = float(line_arr[9])
					msg.MagField.magnetic_field.x = float(line_arr[4])
					msg.MagField.magnetic_field.y = float(line_arr[5])
					msg.MagField.magnetic_field.z = float(line_arr[6])
					msg.imu_data_raw = line_str
					rospy.loginfo("Publishing coordinates")
					imu_pub.publish(msg)
			
	except rospy.ROSInterruptException:	
		port.close()

	except serial.serialutil.SerialException:
		rospy.loginfo("Shutting down IMU node...")













