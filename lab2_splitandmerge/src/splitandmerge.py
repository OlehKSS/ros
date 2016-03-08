#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
from probabilistic_lib.functions import angle_wrap

#===============================================================================
def splitandmerge(points, split_thres=0.1, inter_thres=0.3, min_points=6, dist_thres=0.12, ang_thres=np.deg2rad(10)):
    '''
    Takes an array of N points in shape (2, N) being the first row the x
    position and the second row the y position.

    Returns an array of lines of shape (L, 4) being L the number of lines, and
    the columns the initial and final point of each line in the form
    [x1 y1 x2 y2].

    split_thres: distance threshold to provoke a split
    inter_thres: maximum distance between consecutive points in a line
    min_point  : minimum number of points in a line
    dist_thres : maximum distance to merge lines
    ang_thres  : maximum angle to merge lines
    '''
    lines = split(points, split_thres, inter_thres, min_points, 0, points.shape[1]-1)
    return merge(lines, dist_thres, ang_thres)
    
#===============================================================================
def split(points, split_thres, inter_thres, min_points, first_pt, last_pt):
    '''
    Find lines in the points provided.
    first_pt: column position of the first point in the array
    last_pt : column position of the last point in the array
    '''
    assert first_pt >= 0
    assert last_pt <= points.shape[1]
    
    # TODO: CODE HERE!!!
    # Check minimum number of points
    #if ... < min_points:
        #return None

    # Line defined as "a*x + b*y + c = 0"
    # Calc (a, b, c) of the line (prelab question)
    x1 = points[0, first_pt]
    y1 = points[1, first_pt]
    x2 = points[0, last_pt]
    y2 = points[1, last_pt]
    #
    #

    # Distances of points to line (prelab question)
    #
    #
    #

    # Check split threshold
    #if ... > split_thres:
       
        # Check sublines
        #
        #prev = split(points, split_thres, inter_thres, min_points, ...)
        #post = split(points, split_thres, inter_thres, min_points, ...)
       
        # Return results of sublines
        #if prev is not None and post is not None:
            #return np.vstack((prev, post))
        #elif prev is not None:
            #return prev
        #elif post is not None:
            #return post
        #else:
            #return None

    # Do not need to split furthermore
    #else:
        # Optional check interpoint distance
        #for i in range(first_pt, last_pt):
            #
            #
            #
            #
            # Check interpoint distance threshold
            #if ... > inter_thres:
                #Split line
                #prev = split(points, split_thres, inter_thres, min_points, ...)
                #post = split(points, split_thres, inter_thres, min_points, ...)
               
                # Return results of sublines
                #if prev is not None and post is not None:
                    #return np.vstack((prev, post))
                #elif prev is not None:
                    #return prev
                #elif post is not None:
                    #return post
                #else:
                    #return None
        
        # It is a good line
        #return np.array([[x1, y1, x2, y2]])
        
    # Dummy answer --> delete it
    return np.array([[x1, y1, x2, y2]])

#===============================================================================
def merge(lines, dist_thres, ang_thres):
    '''
    Merge similar lines according to the given thresholds.
    '''
    # No data received
    if lines is None:
        return np.array([])
        
    # Check and merge similar consecutive lines
    #i = 0
    #while i in range(...):
        
        # Line angles
        #ang1 = 
        #ang2 = 
        
        # Below thresholds?
        #angdiff = 
        #disdiff = 
        #if angdiff < ang_thres and disdiff < dist_thres:
            
            # Joined line
            #lines[i,:] = 
            
            # Delete unnecessary line
            #lines = np.delete(lines, ...)
            
        # Nothing to merge
        #else:
            #i += 1
            
    return lines
