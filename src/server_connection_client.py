#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import mavros
import threading
import actionlib
from msg_pkg.srv import checkups
from msg_pkg.msg import ui_checkups_msg

global timer

def timeout():
	#Publish the ping for the UI to see
	checkups_msg = ui_checkups_msg()
	checkups_msg.pi_connect = False
	checkups_msg.px4_connect = False
	checkups_pub.publish(checkups_msg)


def serviceCall():
	rospy.wait_for_service('check_ups')
	try:
		# Call the service on the pi to return connection status
		connection_checkups = rospy.ServiceProxy('check_ups', checkups)
		result = connection_checkups(True)

		#Publish the ping for the UI to see
		checkups_msg = ui_checkups_msg()
		checkups_msg.pi_connect = result.pi_connect
		checkups_msg.px4_connect = result.px4_connect
		checkups_pub.publish(checkups_msg)

		print("Service request successful. Connection status:")
		print(result.pi_connect)
		print(result.px4_connect)

		timer.cancel()
		timer = threading.Timer(5,timeout)
		timer.start()

	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)

		#Publish the ping for the UI to see
		checkups_msg = ui_checkups_msg()
		checkups_msg.pi_connect = False
		checkups_msg.px4_connect = False
		checkups_pub.publish(checkups_msg)

		timer.cancel()
		timer = threading.Timer(5,timeout)
		timer.start()




if __name__ == '__main__':
	rospy.init_node('server_connection_client')
	checkups_pub = rospy.Publisher('connection_checkups', ui_checkups_msg, queue_size=10)
	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		timer = threading.Timer(5,timeout) # If 5 seconds elapse, call timeout()
    	timer.start()
		serviceCall()
		rospy.sleep(1)