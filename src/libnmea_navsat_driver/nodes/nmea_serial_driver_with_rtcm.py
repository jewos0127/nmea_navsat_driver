import serial
import sys
import socket

from base64 import b64encode
#from threading import Thread

import rospy

from libnmea_navsat_driver.driver import RosNMEADriver


def main():
    rospy.init_node('nmea_serial_driver')

    # setting for serial
    serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
    serial_baud = rospy.get_param('~baud', 4800)
    frame_id = RosNMEADriver.get_frame_id()
    # setting for ntrip
    ntrip_server = rospy.get_param('~ntrip_server', 'www.ntrip.com')
    ntrip_mountpoint = rospy.get_param('~ntrip_mountpoint', 'ntrip')
    ntrip_port = rospy.get_param('~ntrip_port', 2101)
    ntrip_user = rospy.get_param('~ntrip_user', 'ntrip')
    ntrip_pass = rospy.get_param('~ntrip_pass', 'ntrip')


    try:
        GPS = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=2)

        try:
            driver = RosNMEADriver()
            while not rospy.is_shutdown():
                data = GPS.readline().strip()
                try:
                    driver.add_sentence(data, frame_id)
                except ValueError as e:
                    rospy.logwarn(
                        "Value error, likely due to missing fields in the NMEA message. "
                        "Error was: %s. Please report this issue at "
                        "github.com/ros-drivers/nmea_navsat_driver, including a bag file with the NMEA "
                        "sentences that caused it." %
                        e)

        except (rospy.ROSInterruptException, serial.serialutil.SerialException):
            GPS.close()  # Close GPS serial port
    except serial.SerialException as ex:
        rospy.logfatal(
            "Could not open serial port: I/O error({0}): {1}".format(ex.errno, ex.strerror))
