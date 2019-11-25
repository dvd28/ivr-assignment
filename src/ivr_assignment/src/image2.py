#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
import pickle
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    self.positions_pub2 = rospy.Publisher("joints_positions2",Float64MultiArray, queue_size = 10)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()


#----------------------------------
  def detect_red2(self,image):
      mask1 = cv2.inRange(cv2.cvtColor(
        image, cv2.COLOR_BGR2HSV), (177, 150, 20), (180, 255, 255))
      mask2 = cv2.inRange(cv2.cvtColor(
        image, cv2.COLOR_BGR2HSV), (0, 150, 20), (10, 255, 255))
      mask = mask1 + mask2
      # This applies a dilate that makes the binary region larger (the more iterations the larger it becomes)
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      # Obtain the moments of the binary image
      M = cv2.moments(mask)
      # Calculate pixel coordinates for the centre of the blob
      cx = int(M['m10'] / M['m00'])
      cz = int(M['m01'] / M['m00'])
      return np.array([cx, cz])
 

  # Detecting the centre of the green circle
  def detect_green2(self,image):
      mask = cv2.inRange(image, (0, 50, 0), (50, 255, 50))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cz = int(M['m01'] / M['m00'])
      return np.array([cx, cz])


  # Detecting the centre of the blue circle
  def detect_blue2(self,image):
      mask = cv2.inRange(image, (50, 0, 0), (255, 50, 50))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cz = int(M['m01'] / M['m00'])
      return np.array([cx, cz])

  # Detecting the centre of the yellow circle
  def detect_yellow2(self,image):
      mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), (27, 180, 50), (33, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cz = int(M['m01'] / M['m00'])
      return np.array([cx, cz])


  # Calculate the conversion from pixel to meter
  def pixel2meter2(self,image):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_blue2(image)
      circle2Pos = self.detect_green2(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 3 / np.sqrt(dist)

  
  #--------------------------------------------
  def detect_joint_positions2(self,image):
    a = self.pixel2meter2(image)
    # Obtain the centre of each coloured blob 
    center = a * self.detect_yellow2(image)
    circle1Pos = a * self.detect_blue2(image) 
    circle2Pos = a * self.detect_green2(image) 
    circle3Pos = a * self.detect_red2(image)

    # make joint position into 2D array(Position array)
    return np.array([[center[0],0.0,center[1]],[circle1Pos[0],0.0,circle1Pos[1]],[circle2Pos[0],0.0,circle2Pos[1]],[circle3Pos[0],0.0,circle3Pos[1]] ])


  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    d = self.detect_joint_positions2(cv_image2)

 
    im2=cv2.imshow('window2',cv_image2)
    cv2.waitKey(3)

    self.positions2 = Float64MultiArray()
    self.positions2.data=d
    


    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(cv_image2, "bgr8"))
      self.positions_pub2.publish(self.positions2)
      print(self.positions2)
      with open('positions2.txt', 'wb') as fp:
        pickle.dump(self.positions2, fp)

    except CvBridgeError as e:
      print(e)



# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


