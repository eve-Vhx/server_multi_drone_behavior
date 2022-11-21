#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import mavros
import random


def gps_publisher():
    rospy.init_node('fake_gps_data', anonymous=True)
    gps1_pub = rospy.Publisher('drone1_gps', NavSatFix, queue_size=10)
    gps2_pub = rospy.Publisher('drone2_gps', NavSatFix, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        gps1_data = NavSatFix()
        gps1_data.latitude = random.uniform(30, 31)
        gps1_data.longitude = random.uniform(-90, -91)
        gps1_data.altitude = random.uniform(240, 242)

        gps1_pub.publish(gps1_data)

        gps2_data = NavSatFix()
        gps2_data.latitude = random.uniform(30, 31)
        gps2_data.longitude = random.uniform(-90, -91)
        gps2_data.altitude = random.uniform(240, 242)

        gps2_pub.publish(gps2_data)
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher()

    except rospy.ROSInterruptException:
        pass