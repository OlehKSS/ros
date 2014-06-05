#!/usr/bin/python
# -*- coding: utf-8 -*-

# Basic imports
import roslib
roslib.load_manifest('lab3_particlefilter')
import rospy
import tf

# ROS messages
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# Maths
import numpy as np

# Transforms
from tf.transformations import euler_from_quaternion

# Custom message
from probabilistic_basics.msg import IncrementalOdometry2D

# Custom libraries
from probabilistic_lib.functions import publish_lines, get_map, get_particle_msgs, yaw_from_quaternion, angle_wrap
from particle_filter import ParticleFilter

#===============================================================================
class LocalizationNode(object):
    '''
    Class to hold all ROS related transactions to use split and merge algorithm.
    '''
    
    #===========================================================================
    def __init__(self, odom_lin_sigma=0.025, odom_ang_sigma=np.deg2rad(2),
                        meas_rng_noise=0.2,  meas_ang_noise=np.deg2rad(10)):
        '''
        Initializes publishers, subscribers and the particle filter.
        '''
        # Publishers
        self.pub_lines = rospy.Publisher("lines", Marker)
        self.pub_particles = rospy.Publisher("particles", PoseArray)
        self.pub_big_particle = rospy.Publisher("mean_particle", PoseStamped)
        self.pub_odom = rospy.Publisher("mean_particle_odom", Odometry)
        
        # Subscribers
        self.sub_scan = rospy.Subscriber("lines", Marker, self.laser_callback)
        self.sub_odom = rospy.Subscriber("incremental_odom", IncrementalOdometry2D, self.odom_callback)
        
        # TF
        self.tfBroad = tf.TransformBroadcaster()
        
        # Incremental odometry
        self.last_odom = None
        self.odom = None
        
        # Flags
        self.new_odom = False
        self.new_laser = False
        self.pub = False
        self.time = None
        
        # Particle filter
        self.part_filter = ParticleFilter(get_map(), 500, odom_lin_sigma,                   odom_ang_sigma, meas_rng_noise, meas_ang_noise)
    
    #===========================================================================
    def odom_callback(self, msg):
        '''
        Saves the incremental odometry of the robot.
        '''
        # Save time
        self.time = msg.header.stamp
            
        # Odometry seen from vehicle frame
        self.uk = np.array([msg.delta_x, msg.delta_y, msg.delta_a])
        
        # Flag
        self.new_odom = True
    
    #===========================================================================
    def laser_callback(self, msg):
        '''
        Reads lines coming from split and merge node. 
        '''
        # Save time
        self.time = msg.header.stamp
        
        # Assertion
        assert msg.type == Marker.LINE_LIST
        
        # Retrieve lines from split and merge
        line = list()
        for point in msg.points:
            line.append(point.x)
            line.append(point.y)
        self.lines = np.array(line).reshape((-1, 4))

        # Have valid points
        if self.lines is not None:
            
            # Flag
            self.new_laser = True
    
    #===========================================================================
    def iterate(self):
        '''
        Main loop of the filter.
        '''
        # Prediction
        if self.new_odom:
            
            self.part_filter.predict(self.uk.copy())
            self.new_odom = False
            self.pub = True
            
        # Weightimg and resampling
        if self.new_laser:
            
            self.part_filter.weight(self.lines.copy())
            self.part_filter.resample()
            self.new_laser = False
            self.pub = True
            
        # Publish results
        if self.pub:
            self.publish_results()
            self.pub = False
    
    #===========================================================================
    def publish_results(self):
        '''
        Publishes all results from the filter.
        '''
        if self.time is not None:
            # Map of the room
            map_lines = get_map()
            publish_lines(map_lines, self.pub_lines, frame='/world', ns='map',
                          color=(0,1,0))
            
            # Particles and biggest weighted particle
            msg, msg_mean, msg_odom, trans, rot = get_particle_msgs(self.part_filter,
                                                                    self.time)
            self.pub_particles.publish(msg)
            self.pub_big_particle.publish(msg_mean)
            self.pub_odom.publish(msg_odom)
            self.tfBroad.sendTransform(translation = trans,
                                       rotation = rot, 
                                       time = self.time,
                                       parent = 'world',
                                       child = 'mean_particle')
                                       
            # Publish scanned lines
            if self.lines is not None:
                publish_lines(self.lines, self.pub_lines, frame='mean_particle',
                          time=self.time, ns='scan_lines_mean', color=(0,0,1))
 
#===============================================================================       
if __name__ == '__main__':
    
    # ROS initializzation
    rospy.init_node('particle_filter')
    node = LocalizationNode(odom_lin_sigma = 0.025,
                            odom_ang_sigma = np.deg2rad(2),
                            meas_rng_noise = 0.2,
                            meas_ang_noise = np.deg2rad(10))
    # Filter at 10 Hz
    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
    
        # Iterate filter
        node.iterate()
        r.sleep()
