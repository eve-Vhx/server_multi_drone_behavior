#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
import mavros
import actionlib
import actionlib_tutorials.msg
import math
from behavior_logic.msg import CommandGoal, CommandResult, CommandFeedback


class BehaviorLogic:

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
            "drone_id": data.position_covariance_type
        }

    def nest_data_cb(self, data):
        self.nest_data = {
            "lat": data.latitude,
            "lon": data.longitude,
            "nest_id": data.position_covariance_type
        }

    def cmd_action_server(self, drone_id):

        if (drone_id == 0):
            self.cmd_client = actionlib.SimpleActionClient('d1_cmd_action', behavior_logic.msg.CommandAction)
        elif (drone_id == 1):
            self.cmd_client = actionlib.SimpleActionClient('d2_cmd_action', behavior_logic.msg.CommandAction)

        rospy.loginfo("waiting for the server...")
        self.cmd_client.wait_for_server()
        rospy.loginfo("connected to the server")
        self.goal = behavior_logic.msg.CommandGoal(lat=self.master_cmd["lat"], lon=self.master_cmd["lon"], alt=self.master_cmd["alt"])
        self.cmd_client.send_goal(self.goal)
        rospy.loginfo("goal sent!")
        self.cmd_client.wait_for_result()
        return self.cmd_client.get_result()



        # if (drone_id == 0):
        #     self.cmd_client = actionlib.SimpleActionClient('d1_cmd_action', actionlib_tutorials.msg.FibonacciAction)
        # elif (drone_id == 1):
        #     self.cmd_client = actionlib.SimpleActionClient('d2_cmd_action', actionlib_tutorials.msg.FibonacciAction)

        # rospy.loginfo("waiting for the server...")
        # self.cmd_client.wait_for_server()
        # rospy.loginfo("connected to the server")
        # self.goal = actionlib_tutorials.msg.FibonacciGoal(order=20)
        # self.cmd_client.send_goal(self.goal)
        # rospy.loginfo("goal sent!")
        # self.cmd_client.wait_for_result()
        # return self.cmd_client.get_result()

    def run_behavior_logic(self):
        rospy.loginfo("running checks to see if drone %i can go to nest %i", self.master_cmd["drone_id"], self.nest_data["nest_id"])

        if(self.master_cmd["drone_id"] == 0):
            other_gps = self.d2_gps_data
        elif(self.master_cmd["drone_id"] == 1):
            other_gps = self.d1_gps_data
        else:
            rospy.loginfo("No valid drone detected!")

        if(math.sqrt(((other_gps["lat"] - self.nest_data["lat"])*111139)**2 + ((other_gps["lon"] - self.nest_data["lon"])*111139)**2) < 20):
                rospy.loginfo("Cannot execute mission. Other drone occupies nest.")
                return False

        else:
            rospy.loginfo("Passed checks... Executing mission")
            self.cmd_result = self.cmd_action_server(drone_id=self.master_cmd["drone_id"])
            rospy.loginfo(self.cmd_result)



    def __init__(self, name):
        rospy.init_node('behavior_logic_node', anonymous=True)
        rospy.Subscriber("drone1_gps", NavSatFix, self.d1_gps_cb, )
        rospy.Subscriber("drone2_gps", NavSatFix, self.d2_gps_cb, )
        rospy.Subscriber("master_cmd", NavSatFix, self.master_cmd_cb, )
        rospy.Subscriber("nest_cmd_data", NavSatFix, self.nest_data_cb, )

        rospy.sleep(2)

        self.run_behavior_logic()







if __name__ == '__main__':
    BehaviorLogic(rospy.get_name())
    rospy.spin()