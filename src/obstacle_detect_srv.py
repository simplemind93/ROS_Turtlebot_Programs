#! /usr/bin/env python

"""

Server definition for crash detection which returns the direction in which the robot
has to move to avoid crash.

    obs_callback : Callback function which the server is called and returns response which 
                   of the type Trigger Response
    
    crash_avoid : Function which reads the laser_data from laser_sub class to see if the 
                  obstacle is closer than 0.8 m and calculates whether robot is going to 
                  crash or not.
    
    obs_avoid : Function which reads laser_data from laser_sub and chooses which direction 
                to move based on laser readings by laser_sub class.

"""

import rospy
import time
from std_srvs.srv import Trigger, TriggerResponse
from laser_subscriber import laser_sub

class detect_obs():
    def __init__(self):
        # Initialize the server with default server settings and a custon dictionary to 
        # store the values provided by the laser_sub class.

        self.laserSubObj = laser_sub()

        # Defining the custom crash dictionary to get the values from laser_sub object 
        # with a default value of -9000 for debugging purposes.
        self.obsDetect = {"Left" : -9000, "Front" : -9000, "Right" : -9000}

        self.detect_obs_srv = rospy.Service("detect_obs_server", Trigger, self.obs_callback)
    
    def obs_callback(self,msg):
        # Server callback function which invokes the laser_data function from the laser_sub 
        # class and returns 
        self.obsDetect = self.laserSubObj.laser_data()

        # Creating the response message using TriggerResponse and returning the direction as 
        # a string so that the robot can move in that direction

        response = TriggerResponse()

        response.success = self.crash_avoid()
        response.message = self.obs_avoid()
        
        return response
    
    def crash_avoid(self):
        # Function which judges if the crash can be avoided or not based on the lidar reading 
        # being greater or lesser than 0.8
        if self.obsDetect["Front"] < 1.0:
            return False
            rospy.logdebug("obstacle_detect_srv.py : Obstacle cannot be avoided!")
        else:
            return True
            rospy.logdebug("obstacle_detct_srv.py : Obstacle can be avoided.")
    
    def obs_avoid(self):
        # parsing the dictionary returned by the laser_data function to see which direction to turn.
        # Directions:
        # 1 - Front
        # 2 - Right
        # 3 - Left

        # if self.crashDict["Left"] == -9000 or self.crashDict["Front"] == -9000 or self.crashDict["Right"] == -9000:
            # rospy.signal_shutdown("obstacle_detect_srv.py : Server dictionary has default values. ")
        
        if self.obsDetect["Front"] > 1.0:
            # if obstacle is more than 1.5 continue forward
            rospy.logdebug("obstacle_detect.py : Continuing forward")
            return "Front"
        else:
            if self.obsDetect["Left"] > self.obsDetect["Right"]:
                rospy.logdebug("obstacle_detect.py : Turning Left")
                return "Left"
            else:
                rospy.logdebug("obstacle_detect.py : Turning Right")
                return "Right"
        
    
if __name__ == "__main__":
    rospy.init_node("obstacle_detect_srv_node", log_level= rospy.DEBUG)
    dObs = detect_obs()
    rospy.spin()