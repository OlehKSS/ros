#!/usr/bin/env python
import rospy

import sys  # command line arguments argv
import math  # atan2

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleWaipoint(object):
    """Class to guide the turtle to the specified waypoint."""

    def __init__(self, waypoint_x=None, waypoint_y=None):
        """Class constructor."""
        # Init all variables
        # Current turtle position
        self.x = None
        self.y = None
        self.theta = None
        # Tolerance to reach waypoint
        self.tolerance = 0.1
        # A position was received
        self.got_position = False
        # Reached position
        self.finished = False
        self.abs_vel = 0.75

        self.default_x = rospy.get_param('/default_x', default=None)
        self.default_y = rospy.get_param('/default_y', default=None)

        # ROS init
        rospy.init_node('turtle_waypoint')
        self.pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/turtle1/pose', Pose, self.callback)

        self.vel = Twist()
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0

        # Retrieve waypoint
        self.waypoint_x = waypoint_x
        self.waypoint_y = waypoint_y

        if waypoint_x is None or waypoint_y is None:
            # No waypoint specified => look at the param server
            if self.default_x is not None and self.default_y is not None:
                print("Waypoint found in param server")

                self.waypoint_x = self.default_x
                self.waypoint_y = self.default_y
            else:
                # No waypoint in param server => finish
                print("No waypoint found in param server")
                exit(1)
        # Show the waypoint

        print('Heading to: {:.2f}, {:.2f}'.format(self.waypoint_x,
                                                  self.waypoint_y))

    def callback(self, msg):
        """Saves the tutle position when a message is received."""
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

        if not self.got_position:
            print("Current position x={0}, y={1}, theta={2}".format(self.x, self.y, self.theta))

        self.got_position = True

    def iterate(self):
        """Keeps sending control commands to turtle until waypoint reached."""
        if self.finished:
            print('Waypoint reached')
            exit(0)
        else:
            # We know where we are
            if self.got_position:
                dist = math.sqrt((self.waypoint_x - self.x) * (self.waypoint_x - self.x) + 
                                (self.waypoint_y - self.y) * (self.waypoint_y - self.y))
                theta = math.atan2((self.waypoint_y - self.y), (self.waypoint_x - self.x))

                if dist < 0.1:
                    self.vel.linear.x = 0
                    self.vel.angular.z = 0
                    self.pub.publish(self.vel)
                    self.finished = True
                    # if we press Ctrl+C the node will stop
                    rospy.spin()
                else:
                    self.vel.linear.x = self.abs_vel
                    self.vel.angular.z = 4 * (theta - self.theta)
                    self.pub.publish(self.vel)


if __name__ == '__main__':
    # Check commandline inputs
    if not len(sys.argv) == 3:
        # No input waypoint specified
        print('No waypoint specified in commandline')
        node = TurtleWaipoint()
    else:
        node = TurtleWaipoint(float(sys.argv[1]), float(sys.argv[2]))
    # Run forever
    while not rospy.is_shutdown():
        node.iterate()
        rospy.sleep(0.05)
    print('\nROS shutdown')
