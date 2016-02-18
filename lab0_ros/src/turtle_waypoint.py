#!/usr/bin/env python
import roslib; roslib.load_manifest('lab0_ros')
import rospy

#For command line arguments
import sys
#For atan2
import math

#TODO: Import the messages we need
##
##

#Initialization of turtle position
x=None
y=None
theta=None

#Position tolerance for both x and y
tolerance=0.1
#Have we received any pose msg yet?
gotPosition=False

def callback(pose_msg):
    global x,y,theta,gotPosition
    #TODO:Store the position in x,y and theta variables.
    ##
    ##
    ##
    ##
    ##
    gotPosition=True

def waypoint():
    global gotPosition
    #TODO: Define the pulisher: Name of the topic. Type of message
    #

    #Name of the node
    rospy.init_node('turtle_waypoint')
    #TODO: Define the subscriber: Name of the topic. Type of message. Callback function
    #

    #Has the turtle reach position?
    finished=False
    #If the point hasn't been specified in a command line:
    if(len(sys.argv)!=3):
        print('X and Y values not set or not passed correctly. Looking for default parameters.')
        #TODO: If ROS parameters default_x and default_y exist:
        if True: #Change this for the correct expression
            #TODO: Save them into this variables
            x_desired=0 #Change this for the correct expression
            y_desired=0 #Change this for the correct expression
            print('Heading to: %f,%f' %(x_desired, y_desired))
        else:
            print('Default values parameters not set!. Not moving at all')
            finished=true
    else:
        #Save the command line arguments.
        x_desired=float(sys.argv[1])
        y_desired=float(sys.argv[2])
        print('Heading to: %f,%f' %(x_desired, y_desired))

    while not rospy.is_shutdown() and not finished:
        if(gotPosition):
            #TODO: Send a velocity command for every loop until the position is reached within the tolerance.
            #
            #
            #
            #
            #
            #
            #
            #
            #
            if True: #Change for the correct expression
                finished=True        

        #Publish velocity commands every 0.3 sec.
        rospy.sleep(0.3)

if __name__ == '__main__':
    try:
        waypoint()
    except rospy.ROSInterruptException:
        pass
