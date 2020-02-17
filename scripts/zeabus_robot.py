#!/usr/bin/env python2
# FILE			: zeabus_robot.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, October 08 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import numpy
import math

cos_45 = math.cos( math.radians( 45 ) )
sin_45 = math.sin( math.radians( 45 ) )

# set direction of thuster
direction_linear = numpy.array([
    [0 , 0 , 1],
    [0 , 0 , 1],
    [0 , 0 , 1],
    [0 , 0 , 1],
    [cos_45, -sin_45 , 0 ],
    [cos_45, sin_45, 0 ],
    [-cos_45, -sin_45 , 0 ],
#    [-cos_45, sin_45 , 0 ]
    [0 , 0 , 0]
])

# set ditance from cg to individual thruster
distance = numpy.array([
    [0.44, 0.32, -0.023],    # thruster id 0
    [0.44, -0.32, -0.023],   # thruster id 1
    [-0.44, 0.32, -0.023],   # thruster id 2
    [-0.44, -0.32, -0.023],  # thruster id 3
    [0.46, 0.34, -0.023],   # thruster id 4
    [0.46, -0.34, -0.023],  # thruster id 5
    [-0.46, 0.34, -0.023],  # thruster id 6
    [-0.46, -0.34, -0.023]  # thruster id 7 
]) 

# mement of force direction
angular = numpy.array( [
    numpy.cross(distance[0], direction_linear[0]),
    numpy.cross(distance[1], direction_linear[1]),
    numpy.cross(distance[2], direction_linear[2]),
    numpy.cross(distance[3], direction_linear[3]),
    numpy.cross(distance[4], direction_linear[4]),
    numpy.cross(distance[5], direction_linear[5]),
    numpy.cross(distance[6], direction_linear[6]),
    numpy.cross(distance[7], direction_linear[7])
] )

direction = numpy.concatenate(
    ( direction_linear , angular ), axis=1 )

direction_inverse = numpy.linalg.pinv( direction )
