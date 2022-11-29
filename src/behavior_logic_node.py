#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import mavros
import actionlib
import actionlib_tutorials.msg
import math
# from behavior_logic.msg import CommandGoal, CommandAction, CommandResult, CommandFeedback
from msg_pkg.msg import server_px4_reqGoal, server_px4_reqAction, server_px4_reqResult, server_px4_reqFeedback 


class BehaviorLogic:

    ######### Callbacks for subscribers ###############
    # handle incoming GPS and command data and store to member variables

    def d1_gps_cb(self, data):
        self.d1_gps_data = {
            "lat": data.latitude,
            "lon": data.longitude,
            "alt": data.altitude
        }
        # rospy.loginfo("D1 Lat: %f", self.d1_gps_data['lat'])


    def d2_gps_cb(self, data):
        self.d2_gps_data = {
            "lat": data.latitude,
            "lon": data.longitude,
            "alt": data.altitude
        }
        # rospy.loginfo("D2 Lat: %f", self.d2_gps_data['lat'])

    def master_cmd_cb(self, data):
        self.master_cmd = {
            "lat": data.latitude,
            "lon": data.longitude,
            "alt": data.altitude,
            "drone_id": data.position_covariance_type,
            "yaw_rad": 0,
            "mission_type": 1,
        }

    #####################################################
    ########## Helper functions #########################

    def convert_deg_to_m(self, lat1, lat2, lon1, lon2):
        # Convert differences in latitude and longitude to distance in meters
        R = 6371000
        phi1 = lat1*math.pi/180
        phi2 = lat2*math.pi/180
        d_phi = (lat2-lat1)*math.pi/180
        d_delta = (lon2-lon1)*math.pi/180

        a = math.sin(d_phi/2)*math.sin(d_phi/2) + math.cos(phi1)*math.cos(phi2)*math.sin(d_delta/2)*math.sin(d_delta/2)
        c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
        d = R*c #distance in meters
        rospy.loginfo("Distance in meters between drone and nest: %f", d)
        return d 

    #####################################################
    ########### Run the logic ###########################

    def cmd_action_server(self, drone_id):
        # Call the action server to the rpi to send mission to px4
        if (drone_id == 0):
            self.cmd_client = actionlib.SimpleActionClient('d1_cmd_action', server_px4_reqAction)
        elif (drone_id == 1):
            self.cmd_client = actionlib.SimpleActionClient('d2_cmd_action', server_px4_reqAction)

        rospy.loginfo("waiting for the server...")
        self.cmd_client.wait_for_server()
        rospy.loginfo("connected to the server")
        self.goal = server_px4_reqGoal(lat=self.master_cmd["lat"], lon=self.master_cmd["lon"], alt=self.master_cmd["alt"], yaw_rad=self.master_cmd["yaw_rad"], mission_type=self.master_cmd["mission_type"])
        self.cmd_client.send_goal(self.goal)
        rospy.loginfo("goal sent!")
        self.cmd_client.wait_for_result()
        return self.cmd_client.get_result()


    def run_behavior_logic(self):
        # Check to see if another drone occupies the mission nest
        rospy.loginfo("running checks to see if drone %i can go to nest", self.master_cmd["drone_id"])

        if(self.master_cmd["drone_id"] == 0):
            other_gps = self.d2_gps_data
        elif(self.master_cmd["drone_id"] == 1):
            other_gps = self.d1_gps_data
        else:
            rospy.loginfo("No valid drone detected!")

        #Convert lat/lon distances to meters

        if(self.convert_deg_to_m(self.master_cmd["lat"], other_gps["lat"], self.master_cmd["lon"], other_gps["lon"]) < 10):
            rospy.loginfo("Cannot execute mission. Other drone occupies nest.")
            return False

        else:
            rospy.loginfo("Passed checks... Executing mission")
            self.cmd_result = self.cmd_action_server(drone_id=self.master_cmd["drone_id"])
            rospy.loginfo(self.cmd_result)

    #####################################################
    ########### Initialize class and member variables ###



    def __init__(self, name):
        rospy.init_node('behavior_logic_node', anonymous=True)
        rospy.Subscriber("drone1_gps", NavSatFix, self.d1_gps_cb, )
        rospy.Subscriber("drone2_gps", NavSatFix, self.d2_gps_cb, )
        rospy.Subscriber("master_cmd", NavSatFix, self.master_cmd_cb, )

        rospy.sleep(2)

        self.run_behavior_logic()







if __name__ == '__main__':
    BehaviorLogic(rospy.get_name())
    rospy.spin()