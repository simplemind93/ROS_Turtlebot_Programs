#! /usr/bin/env python

""" 

Class definition for laser_sub.
Class functions:
    laser_sub_callback : Callback function used by the subscriber which stores the value of LaserScan
                         in an laserObj
    
    laser_data : parses the laserObj.ranges array to obtain value of the laser readings at the left,front 
                 and right of the robot for crash detection.

    laser_debug : debugging function which checks if the value of the left, front and right are properly 
                  parsed by comparing it to default value of -999.0

"""

import rospy
import time
from sensor_msgs.msg import LaserScan

class laser_sub():
    def __init__(self):
        #rospy.logdebug('laser_subscriber.py : class initialized')
        self.laserSub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.laser_sub_callback)
        self.laserObj = LaserScan()
        self.rate = rospy.Rate(2)

        # initalizing left, right and front with default values to ehlp with debug if the 
        # subscriber is not working properly.

        self.left = -999.0
        self.right = -999.0
        self.front = -999.0
    
    def laser_sub_callback(self, msg):
        # Callback function which is invoked by the subscriber which extracts the left, center
        # right values from the /kobuki/laser/scan topic for collision detection and navigating
        # out of the maze.
        
        #rospy.logdebug("laser_subscriber.py : callback function initialized")
        self.laserObj = msg
        # rospy.logdebug(self.laserObj)
        # rospy.logdebug(self.laserObj.ranges)
        
    
    def laser_data(self):
        # function used to extract the laserscan data and log it to the user and other programs
        self.left = self.laserObj.ranges[719]
        self.front = self.laserObj.ranges[360]
        self.right = self.laserObj.ranges[0]
        rospy.loginfo('laser_subscriber.py : left ' + str(self.left))
        rospy.loginfo('laser_subscriber.py : front ' + str(self.front))        
        rospy.loginfo('laser_subscriber.py : right ' + str(self.right))
        laserRead = {"Left" : self.left, "Front" : self.front, "Right" : self.right}
        return laserRead
    
    def laser_debug(self):
        # debug function which compares the self.left, fron and eight to the default value which 
        # is -999.0 to check if the subscriber is working properly.

        if self.left == -999.0  or self.right == -999.0  or self.front == -999.0:
            rospy.logerr("!!! laser_subscriber.py : subscriber not working properly !!!")
        else:
            rospy.logdebug("laser_suscriber.py : subscriber working properly")

if __name__ == "__main__":
    rospy.init_node('laser_subscriber_node', log_level = rospy.INFO)
    rospy.logdebug('laser_subscriber.py : launched in debug mode.')
    laserSubObj = laser_sub()
    ctrl_c = False
    rate = rospy.Rate(0.5)
    time.sleep(2)

    def shutdownhook():
        global ctrl_c
        global twist_object
        global pub
        
        rospy.loginfo("laser_subscriber.py : SHUTDOWN TIME")
        ctrl_c =True
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        laser = laserSubObj.laser_data()
        rospy.logdebug(laser)
        rate.sleep()
    
