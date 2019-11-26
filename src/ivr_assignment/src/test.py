#!/usr/bin/env python

import roslib
import sys
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
from robot_formulas import FK

'''
Test FK formula
'''

class Tester:
    
    def __init__(self):
      rospy.init_node('testing', anonymous=True)

      # Subscribe to the detected positions of the joints
      '''
      self.jointpos_sub1 = rospy.Subscriber("joints_positions1",
        Float64MultiArray, self.process_pos1)
      self.jointpos_sub2 = rospy.Subscriber("joints_positions2", 
        Float64MultiArray, self.process_pos2)
      '''

      # Subsrcribe to the joint angle commands topics
      self.joint1_angle_sub = rospy.Subscriber(
        "/robot/joint1_position_controller/command", Float64, self.process_angle1)
      self.joint2_angle_sub = rospy.Subscriber(
        "/robot/joint2_position_controller/command", Float64, self.process_angle2)
      self.joint3_angle_sub = rospy.Subscriber(
        "/robot/joint3_position_controller/command", Float64, self.process_angle3)
      self.joint4_angle_sub = rospy.Subscriber(
        "/robot/joint4_position_controller/command", Float64, self.process_angle4)

      # store joint angle positions
      self.a0 = 0.0
      self.a1 = 0.0
      self.a2 = 0.0
      self.a3 = 0.0

    def process_angle1(self, a):
      self.a0 = a.data
      self.print_epos()

    def process_angle2(self, a):
      self.a1 = a.data
      self.print_epos()    

    def process_angle3(self, a):
      self.a2 = -a.data
      self.print_epos()

    def process_angle4(self, a):
      self.a3 = a.data
      self.print_epos()    


    def print_epos(self):
      print(np.around(FK(self.a0, self.a1, self.a2, self.a3, 2, 3, 2), 2))


# call the class
def main(args):
  t = Tester()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
