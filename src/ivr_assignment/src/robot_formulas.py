#!/usr/bin/env python

from math import sin, cos
import numpy


'''
Calculates the end effector position of the robot given the four joint positions
and the length of the three robot links
'''
def FK(a0, a1, a2, a3, d1, d2, d3):
  a2 = -a2
  x = (sin(a0)*cos(a1)*sin(a3) - cos(a0)*sin(a2)*cos(a3) + \
       sin(a0)*sin(a1)*cos(a2)*cos(a3)) * d3 + (-cos(a0)*sin(a2) + sin(a0)*sin(a1)*cos(a2)) * d2
  y = (-cos(a0)*cos(a1)*sin(a3) - sin(a0)*sin(a2)*cos(a3) - \
       cos(a0)*sin(a1)*cos(a2)*cos(a3)) * d3 + (-sin(a0)*sin(a2) - cos(a0)*sin(a1)*cos(a2)) * d2
  z = (-sin(a1)*sin(a3) + cos(a1)*cos(a2)*cos(a3)) * d3 + cos(a1)*cos(a2)*d2 + d1
  return numpy.array([x, y, z])
      

