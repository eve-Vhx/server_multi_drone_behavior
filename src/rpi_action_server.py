#! /usr/bin/env python

import rospy
from behavior_logic.msg import px4_mission_req
import actionlib
from sensor_msgs.msg import NavSatFix
import actionlib_tutorials.msg
from behavior_logic.msg import CommandAction, CommandResult, CommandFeedback, CommandGoal
from mavros_msgs.msg import state

class DroneAction():
    # create messages that are used to publish feedback/result
    _feedback = CommandFeedback()
    _result = CommandResult()

    def __init__(self):
        self.cmd_pub = rospy.Publisher('smr', px4_mission_req, queue_size=10)
        rospy.Subscriber("drone1_gps", NavSatFix, self.d1_gps_cb, )
        #rospy.Subscriber("mavros/state", state, self.state_msg_cb, )
        self._as = actionlib.SimpleActionServer("d2_cmd_action", CommandAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def convert_deg_to_m(self, lat1, lat2, lon1, lon2):
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

    def state_check(self):
        dist_to_nest = self.convert_deg_to_m(cmd_msg.lat, d1_gps_data["lat"], cmd_msg.lon, d1_gps_data["lon"])

        if (dist_to_nest < 1):
            return True
        else:
            return False
      
    def execute_cb(self, goal):

        cmd_msg = px4_mission_req()
        cmd_msg.lat = goal.lat
        cmd_msg.lon = goal.lon
        cmd_msg.alt = goal.alt
        cmd_msg.mission_type = 1
        cmd_msg.yaw_rad = 0
        
        rospy.loginfo("Publishing mission to px4... Mission details | lat: %f, lon: %f, alt: %f", cmd_msg.lat, cmd_msg.lon, cmd_msg.alt)
        self.cmd_pub.publish(cmd_msg)

        # helper variables
        r = rospy.Rate(1)
        success = False

        while(success == False):
            if (self.state_check()):
                self._result.time_elapsed = rospy.rostime.Duration()
                self._result.mission_completion = 1
                rospy.loginfo("Drone successfully landed in nest. Mission complete.")
                self._as.set_succeeded(self._result)
                success = True
            else:
                continue
        
          

    def d1_gps_cb(self, data):
        self.d1_gps_data = {
            "lat": data.latitude,
            "lon": data.longitude,
            "alt": data.altitude
        }

    # def state_msg_cb(self, data):
    #     self.d1_state = data.mode
        
if __name__ == '__main__':
    rospy.init_node('rpi_action_server')
    server = DroneAction()
    rospy.spin()