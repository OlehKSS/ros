#!/usr/bin/env python
import rospy

import sys  # command line arguments argv
import math  # atan2

# TODO: Import the messages we need
# import ...
# import ...


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

        # ROS init
        rospy.init_node('turtle_waypoint')
        # TODO: Define pulisher: topic name, message type
        # self.pub = rospy.Publisher(...)
        # TODO: Define subscriber: topic name, message type, function callback
        # self.sub = rospy.Subscriber(...)

        # Retrieve waypoint
        self.waypoint_x = waypoint_x
        self.waypoint_y = waypoint_y
        if waypoint_x is None or waypoint_y is None:
            # No waypoint specified => look at the param server
            if False:  # TODO: change for the correct expression
                print("Waypoint found in param server")
                # TODO: Save params from param server
                self.waypoint_x = 0  # TODO: change for the correct expression
                self.waypoint_y = 0  # TODO: change for the correct expression
            else:
                # No waypoint in param server => finish
                print("No waypoint found in param server")
                exit(1)
        # Show the waypoint
        print('Heading to: {:.2f}, {:.2f}'.format(self.waypoint_x,
                                                  self.waypoint_y))

    def callback(self, msg):
        """Saves the tutle position when a message is received."""
        # TODO: store the position in self.x, self.y and self.theta variables.
        # self.x = ...
        # self.y = ...
        # self.theta = ...
        self.got_position = True

    def iterate(self):
        """Keeps sending control commands to turtle until waypoint reached."""
        if self.finished:
            print('Waypoint reached')
            exit(0)
        else:
            # We know where we are
            if self.got_position:
                #
                #
                if True:  # TODO: change for the correct expression
                    # Waypoint reached
                    self.finished = True
                else:
                    # Waypoint not reached yet
                    # TODO: Send a velocity command towards waypoint
                    #
                    math.atan2(1, 0)  # TODO: delete this line
                    #
                    #
                    # self.pub.publish(...)


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
        rospy.sleep(0.3)
    print('\nROS shutdown')
