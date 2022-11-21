#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import mavros


class BehaviorLogic:

    def d1_gps_cb(self, data):
        self.d1_gps_data = {
            "lat": data.latitude,
            "lon": data.longitude,
            "alt": data.altitude
        }
        rospy.loginfo("D1 Lat: %f", self.d1_gps_data['lat'])


    def d2_gps_cb(self, data):
        self.d2_gps_data = {
            "lat": data.latitude,
            "lon": data.longitude,
            "alt": data.altitude
        }
        rospy.loginfo("D2 Lat: %f", self.d2_gps_data['lat'])

    def master_cmd_cb(self, data):
        self.master_cmd = {
            "lat": data.latitude,
            "lon": data.longitude,
            "alt": data.altitude,
            "drone_id": data.position_covariance_type
        }

    def __init__(self):
        rospy.init_node('behavior_logic_node', anonymous=True)
        rospy.Subscriber("drone1_gps", NavSatFix, self.d1_gps_cb, )
        rospy.Subscriber("drone2_gps", NavSatFix, self.d2_gps_cb, )
        rospy.Subscriber("master_cmd", NavSatFix, self.master_cmd_cb, )
        rospy.spin()




if __name__ == '__main__':
    BehaviorLogic()