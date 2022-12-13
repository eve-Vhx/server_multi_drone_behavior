#!/usr/bin/env python

from msg_pkg.srv import UiReq
import rospy

def handle_request(req):
    print("Lat: %f, Lon: %f, Alt: %f", req.lat, req.lon, req.alt)
    return 1

def ui_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('ui_mission_req', UiReq, handle_request)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    ui_server()