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
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    self.positions_pub1 = rospy.Publisher("joints_positions1",Float64MultiArray, queue_size = 10)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
   


#----------------------------------
  def detect_red1(self,image):
      # This also detect red when the semi-transparent orange target is in front of it
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
      cz = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cz, cy])
 

  # Detecting the centre of the green circle
  def detect_green1(self,image):
      mask = cv2.inRange(image, (0, 50, 0), (50, 255, 50))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cz = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cz, cy])


  # Detecting the centre of the blue circle
  def detect_blue1(self,image):
      mask = cv2.inRange(image, (50, 0, 0), (255, 50, 50))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cz = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cz, cy])

  # Detecting the centre of the yellow circle
  def detect_yellow1(self,image):
      mask = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), (27, 180, 50), (33, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cz = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cz, cy])


  # Calculate the conversion from pixel to meter
  def pixel2meter1(self,image):
      # Obtain the centre of each coloured blob
      circle1Pos = self.detect_blue1(image)
      circle2Pos = self.detect_green1(image)
      # find the distance between two circles
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 3 / np.sqrt(dist)

  
  #--------------------------------------------
  def detect_joint_positions1(self,image):
    a = self.pixel2meter1(image)
    # Obtain the centre of each coloured blob 
    yellow = a * self.detect_yellow1(image)
    blue = a * self.detect_blue1(image) 
    green = a * self.detect_green1(image) 
    red = a * self.detect_red1(image)
#coverting image pos to 3D cordinate(origin as yellow)
    yellow_y=yellow[0]
    yellow_z=yellow[1]
    yellow[1]=yellow_z-yellow[1]+0.00000000001
    blue[1]=yellow_z-blue[1]+0.00000000002
    green[1]=yellow_z-green[1]+0.00000000003
    red[1]=yellow_z-red[1]+0.00000000004

    yellow[0]=yellow[0]-yellow_y+0.00000000001
    blue[0]=blue[0]-yellow_y+0.00000000002
    green[0]=green[0]-yellow_y+0.00000000003
    red[0]=red[0]-yellow_y+0.00000000004


#--------------------------------------------------
    # make joint position into 2D array(Position array)
    return np.array([[0.0,yellow[0],yellow[1]],[0.0,blue[0],blue[1]],[0.0,green[0],green[1]],[0.0,red[0],red[1]] ])




  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)



    # some testing 
    # Testing different yellow ranges
    # 0:
    # image_test = cv2.inRange(cv_image1, (0, 100, 100), (0, 255, 255))
    # 1:
    # image_test = cv2.inRange(cv_image1, (0, 50, 50), (50, 255, 255)
    # 2:
    # image_test = cv2.inRange(cv2.cvtColor(
    #  cv_image1, cv2.COLOR_BGR2HSV), (27, 180, 50), (33, 255, 255))

    # Testing different blue ranges
    # 0:
    # image_test = cv2.inRange(cv_image1, (100, 0, 0), (255, 0, 0))
    # 1:
    # image_test = cv2.inRange(cv_image1, (50, 0, 0), (255, 50, 50))

    # Testing different green ranges
    # 0:
    # image_test = cv2.inRange(cv_image1, (0, 100, 0), (0, 255, 0))
    # 1:
    # image_test = cv2.inRange(cv_image1, (0, 50, 0), (50, 255, 50))

    # Testing different red ranges
    # 0:
    # image_test = cv2.inRange(cv_image1, (0, 0, 100), (0, 0, 255))
    # 1:
    # image_test = cv2.inRange(cv_image1, (0, 0, 50), (50, 50, 255))
    # 2:
    '''
    image_test1 = cv2.inRange(cv2.cvtColor(
      cv_image1, cv2.COLOR_BGR2HSV), (177, 150, 20), (180, 255, 255))
    image_test2 = cv2.inRange(cv2.cvtColor(
      cv_image1, cv2.COLOR_BGR2HSV), (0, 150, 20), (10, 255, 255))
    image_test = image_test1 + image_test2
    cv2.imwrite('red2.png', image_test)
    '''
    #cv2.imwrite('test_image.png', cv_image1)


    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    
#-----------------FALUTY Z AXIS(inverse it)---------------

#---------------------------------------------

    #im1=cv2.imshow('window1',cv_image1)
    #cv2.waitKey(3)


 
    b = self.detect_joint_positions1(cv_image1)
    self.positions1 = Float64MultiArray()
    self.positions1.data=b
    


    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(cv_image1, "bgr8"))
      self.positions_pub1.publish(self.positions1) 
      #print(self.positions1.data)


      with open('positions1.txt', 'wb') as fp:
        pickle.dump(self.positions1, fp)

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


