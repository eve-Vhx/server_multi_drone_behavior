#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import mavros
import random


def cmd_publisher():
    rospy.init_node('fake_cmd_data', anonymous=True)
    master_cmd_pub = rospy.Publisher('master_cmd', NavSatFix, queue_size=10)
    nest_data_pub = rospy.Publisher('nest_cmd_data', NavSatFix, queue_size=10)
    rate = rospy.Rate(10)

    # cmd_lat = input('Desired Lat: ')
    # cmd_lon = input('Desired Lon: ')
    # cmd_alt = input('Desired Alt: ')
    # drone_id = input('Desired Drone ID: ')

    cmd_lat = 30.1234534
    cmd_lon = -90.2532567
    cmd_alt = 236.1256742
    drone_id = 1

    nest_lat = 31.2144321
    nest_lon = -91.2334321
    nest_id = 1

    cmd_msg = NavSatFix()
    cmd_msg.latitude = cmd_lat
    cmd_msg.longitude = cmd_lon
    cmd_msg.altitude = cmd_alt
    cmd_msg.position_covariance_type = drone_id

    nest_cmd = NavSatFix()
    nest_cmd.latitude = nest_lat
    nest_cmd.longitude = nest_lon
    nest_cmd.position_covariance_type = nest_id

    while not rospy.is_shutdown():

        master_cmd_pub.publish(cmd_msg)
        nest_data_pub.publish(nest_cmd)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        cmd_publisher()

    except rospy.ROSInterruptException:
        pass