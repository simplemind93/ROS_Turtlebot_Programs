#! /usr/bin/env python
"""
Class definition for class odo_sub.


"""

import rospy
import time
from nav_msgs.msg import Odometry

class odo_sub():
    
    def __init__(self):
        rospy.logdebug("odod_subscriber.py : class initialized")
        self.odoSub = rospy.Subscriber("/odom", Odometry, self.odo_sub_callback)
        self.odoObj = Odometry()
        self.odoObj.header.frame_id = "debug"
        self.rate = rospy.Rate(1)
    
    def odo_sub_callback(self, msg):
        # Subscriber callback function to store the returned message in the odoObj

        self.odoObj = msg
        rospy.logdebug(self.odoObj)

    def odo_get_data(self):
        
        return self.odoObj
    
    def odo_debug(self):
        # debug function which tests if the subscriber is working properly
        if self.odoObj.header.frame_id == "debug":
            rospy.logerr("!!! odo_subscriber.py : SUSCRIBER NOT WORKING PROPERLY !!!")
        else:
            rosp.logdebug("odo_subscriber.py : subscriber working as expected")

if __name__ == "__main__":
    rospy.init_node('laser_subscriber_node', log_level = rospy.DEBUG)
    rospy.logdebug(" odo_subscriber.py : launched in debug mode.")
    odoSubObj = odo_sub()
    ctrl_c = False
    rate = rospy.Rate(1)
    # this is sleep command is needed for the subscriber to initalize and obtain messages before the program is run.
    time.sleep(2)

    def shutdownhook():
        global ctrl_c
        global twist_object
        global pub
        
        rospy.loginfo("odo_subscriber.py : SHUTDOWN TIME")
        ctrl_c =True
    
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        odoData = odoSubObj.odo_get_data()
        rospy.logdebug(odoData)
        rate.sleep()