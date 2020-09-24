#! /usr/bin/env python
"""
Class definition for the class velpublisher.
class functions:
    pub_move_cmd_once - moves the robot by publishing to the cmd_vel topic once
                        based on the cmd given to represent the direction.

    stop_robot - stops the robot by reducing the velocity to zero and publishing it
                 useful for debugging and during shutdown.

    cmd_debug - Function for testing the class through debugging function.

    db_callback - Callback function for the subscriber which is created by the cmd_debug
                  function.

"""

import rospy
import time
from geometry_msgs.msg import Twist

class velPublisher():

    def __init__(self):
        # defining the publisher and publisher message type
        # for use throught out the class
        rospy.logdebug(" velpublisher.py : Publisher has been initialized")
        self.cmd_publisher = rospy.Publisher('/cmd_vel',Twist,queue_size = 1)
        self.cmd_vel = Twist()
        # Debug_move variable is to be only used when the cmd_debug 
        # is to be called to verify if the publisher class is functioning
        self.debug_move = False
        
    def pub_move_cmd_once(self, cmd):

        # This function publishes to cmd_vel based on the given input
        # The inputs are int32 variables meant to represent the 
        # direction based on the current heading of the robot
        # 1 - move forward
        # 2 - move right
        # 3 - move left
        # 4 - move backward
        
        connections = self.cmd_publisher.get_num_connections()

        if connections > 0:
            if cmd == "Front":
                self.cmd_vel.linear.x = 0.3
                rospy.logdebug('The robot is moving straight.')
            elif cmd == "Right":
                self.cmd_vel.angular.z = -0.5
                rospy.logdebug('The robot is turning left.')
            elif cmd == "Left":
                self.cmd_vel.angular.z = 0.5
                rospy.logdebug('The robot is turning right')
            elif cmd == "Stop":
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 0.0
                rospy.logdebug('The robot is stopping')
            else :
                # The robot going backward is not bery useful but
                # has been implemented incase of the necessity for
                # it should ever arise.
                self.cmd_vel.linear.x = -0.3
                rospy.logdebug('!!! The robot is moving in reverse !!!')
            
            # Publishing to cmd_vel topic after reading the 
            # based on the input
            self.cmd_publisher.publish(self.cmd_vel)
            
    
    def stop_robot(self):
        # Function to stop the robot during debugger
        rospy.loginfo("velpublisher.py : stopping the robot")
        self.cmd_vel.linear.x = 0
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0
        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = 0
        self.cmd_publisher.publish(self.cmd_vel)

    def cmd_debug(self):
        # Function only for debugging and to check if the publisher
        # is properly functioning 
        self.cmd_vel.linear.y = 100
        self.cmd_publisher.publish(self.cmd_vel)
        rospy.logdebug("velpublisher.py : DEBUGGER HAS BEEN CALLED")
        self.db_subscriber = rospy.Subscriber('/cmd_vel', Twist, self.db_callback)
        # Debug values to test if the publsiher is working properly 
        # these values wouldn't move the robot while allowing us to
        # test the publisher.

        if self.db_subscriber:
            rospy.loginfo("velpublisher.py : cmd_vel has been succesfully published")        
        else :
            rospy.logerr("!!! velpublisher.py : CMD_VEL HAS FAILED TO PUBLISH PROPERLY !!!")
    
    def db_callback(self, msg):

        # debug callback function
        if msg.linear.y == 100:
            return True
        else:
            return False

        self.stop_robot()

    
    
if __name__ == "__main__":
    
    # choice = input("1 : normal operation \n 2: debug mode")
    rospy.init_node('publish_vel_cmd')
    vp = velPublisher()
    ctrl_c = False
    rate = rospy.Rate(1)

    def shutdownhook():
        global ctrl_c
        global twist_object
        global pub
        
        rospy.loginfo("velpublisher.py : SHUTDOWN TIME")
        ctrl_c =True
        vp.stop_robot()
    
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        vp.pub_move_cmd_once("Left")
        rate.sleep()
    
        
