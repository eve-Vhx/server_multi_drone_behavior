#! /usr/bin/env python

import rospy
from behavior_logic.msg import px4_mission_req
import actionlib
from sensor_msgs.msg import NavSatFix
import actionlib_tutorials.msg
from behavior_logic.msg import CommandAction, CommandResult, CommandFeedback, CommandGoal

class DroneAction():
    # create messages that are used to publish feedback/result
    _feedback = CommandFeedback()
    _result = CommandResult()

    def __init__(self):
        self.cmd_pub = rospy.Publisher('smr', px4_mission_req, queue_size=10)
        self._as = actionlib.SimpleActionServer("d2_cmd_action", CommandAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
      
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
        success = True
        
          
        if success:
            self._result.time_elapsed = rospy.rostime.Duration()
            self._result.mission_completion = 1
            rospy.loginfo("Action succeeded!")
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('rpi_action_server')
    server = DroneAction()
    rospy.spin()