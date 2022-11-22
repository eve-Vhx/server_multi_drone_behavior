#! /usr/bin/env python

import rospy

import actionlib

import actionlib_tutorials.msg
from behavior_logic.msg import CommandAction, CommandResult, CommandFeedback, CommandGoal

class DroneAction(object):
    # create messages that are used to publish feedback/result
    #_feedback = actionlib_tutorials.msg.FibonacciFeedback()
    _result = CommandResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, CommandAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo("Started the action server")
      
    def execute_cb(self, goal):

        goal_lat = goal.lat
        goal_lon = goal.lon
        goal_alt = goal.alt

        rospy.loginfo("Drone data: %f, %f, %f", goal_lat, goal_lon, goal_alt)


        # helper variables
        r = rospy.Rate(1)
        success = True
        
        # start executing the action
        # for i in range(1, goal.order):
        #     # check that preempt has not been requested by the client
        #     if self._as.is_preempt_requested():
        #         rospy.loginfo('%s: Preempted' % self._action_name)
        #         self._as.set_preempted()
        #         success = False
        #         break
        #     self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
        #     # publish the feedback
        #     self._as.publish_feedback(self._feedback)
        #     # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        #     r.sleep()
          
        if success:
            self._result.success = 1
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)
        
if __name__ == '__main__':
    rospy.init_node('d1_cmd_action')
    server = DroneAction('d1_cmd_action')
    rospy.spin()