#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import utm
import serial
from std_msgs.msg import Header
from gps_driver.msg import gps_msg


def UTC_to_Sec (time):
	t_hr = int(time/10000)
	t_min = int((time - (t_hr*10000))/100)
	t_sec = int(time%100)
	sec = t_hr*3600+t_min*60+t_sec
	nsec = (time%1)*1000000000
	return sec, nsec

def degree_conversion(lat, lon, lat_dir, lon_dir):
	lat_DD = int(lat/100)
	lat_mm = lat - (lat_DD*100)
	if lat_dir == 'N':
		lati = lat_DD + (lat_mm/60)
	else:
		lati = -(lat_DD + (lat_mm/60))
	long_DD = int(lon/100)
	long_mm = lon - (long_DD*100)
	if lon_dir == 'E':
		longi = long_DD + (long_mm/60)
	else:
		longi = - (long_DD + (long_mm/60))
	return lati, longi


if __name__ == '__main__':
	rospy.init_node('GPS_to_UTM', anonymous=True)
	serial_port = rospy.get_param('~port','/dev/ttyUSB0')
	#serial_port = rospy.get_param('~port','/dev/pts/4')
	serial_baud = rospy.get_param('~baudrate',4800)
	sampling_rate = rospy.get_param('~sampling_rate',5.0)
	
	port = serial.Serial(serial_port, serial_baud, timeout=3.)
	rospy.logdebug("Using GPS sensor on port /dev/ttyUSB0 at 4800 baudrate")
	rospy.logdebug("Initializing sensor\\r\\n ...")

	sampling_count = int(round(1/(sampling_rate*0.007913)))
	rospy.sleep(0.2)
	line = port.readline()
	gps_pub = rospy.Publisher('gps', gps_msg, queue_size=10)

	rospy.logdebug("Initialization complete")
	
	msg = gps_msg()

	sleep_time = 1/sampling_rate - 0.025

	try:
		while not rospy.is_shutdown():
			line = port.readline()
			#print(line)
			if line == '':
				rospy.logwarn("GPS: No data")
			else:
				line_str = line.decode('UTF-8')
				line_arr = line_str.split(',')
				#print(line_arr)
				if '$GPGGA' in line_arr[0]:
					second, nano_second = UTC_to_Sec(float(line_arr[1]))
					msg.Header.stamp.secs = int(second)
					msg.Header.stamp.nsecs = int(nano_second)
					msg.Header.frame_id = "GPS1_Frame"
					latitude, longitude = degree_conversion(float(line_arr[2]),float(line_arr[4]),line_arr[3],line_arr[5])
					msg.Latitude = latitude
					msg.Longitude = longitude
					msg.Altitude = float(line_arr[9])
					easting, northing, zone, letter = utm.from_latlon(latitude,longitude)
					msg.UTM_easting = easting
					msg.UTM_northing = northing
					msg.Zone = zone
					msg.Letter = letter
					rospy.loginfo("Publishing coordinates")
					gps_pub.publish(msg)
			rospy.sleep(sleep_time)
			
	except rospy.ROSInterruptException:
		port.close()

	except serial.serialutil.SerialException:
		rospy.loginfo("Shutting down GPS node...")













