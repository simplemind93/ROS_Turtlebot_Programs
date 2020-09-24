#! /usr/bin/env python  
"""

Client definition based on the obstacle_detect_srv.py server definition for testing purposes.



"""

import rospy
import time
from std_srvs.srv import Trigger, TriggerRequest

class obs_client():
    def __init__(self):
        # Initialize the client for the obstacle_detect_srv server

        rospy.logdebug("obstacle_detect_client.py : Client initialized.")
        rospy.wait_for_service("detect_obs_server")
        self.obsClient = rospy.ServiceProxy('detect_obs_server', Trigger)
        self.obsRequest = TriggerRequest()
    
    def client_call(self):
        # Simple program which calls the server with the request message for the server.
        # Which for this server the request message is TriggerRequest which is Empty.

        self.response = self.obsClient(self.obsRequest)
        rospy.logdebug(self.response)

if __name__ == "__main__":
    rospy.init_node('obstacle_detect_client_node', log_level= rospy.DEBUG)
    obsClient = obs_client()
    while not rospy.is_shutdown():
        obsClient.client_call()
        rospy.spin()