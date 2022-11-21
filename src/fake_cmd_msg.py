#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import mavros
import random


def cmd_publisher():
    rospy.init_node('fake_cmd_data', anonymous=True)
    master_cmd_pub = rospy.Publisher('master_cmd', NavSatFix, queue_size=10)
    rate = rospy.Rate(10)

    cmd_lat = input('Desired Lat: ')
    cmd_lon = input('Desired Lon: ')
    cmd_alt = input('Desired Alt: ')
    drone_id = input('Desired Drone ID: ')

    # cmd_lat =
    # cmd_lon =
    # cmd_alt = 
    # drone_id = 

    cmd_msg = NavSatFix()
    cmd_msg.latitude = cmd_lat
    cmd_msg.longitude = cmd_lon
    cmd_msg.altitude = cmd_alt
    cmd_msg.position_covariance_type = drone_id

    while not rospy.is_shutdown():

        master_cmd_pub.publish(cmd_msg)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        cmd_publisher()

    except rospy.ROSInterruptException:
        pass