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

    # initialise a publishers for sending the prediction of y and z coordinates of target
    self.target_ypredict_pub = rospy.Publisher("target/ypredict", Float64, queue_size=1)
    self.target_zpredict_pub = rospy.Publisher("target/zpredict1", Float64, queue_size=1)

    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
   
    # a few things to remember
    self.callback_called = False
    self.pixel_meter_ratio = 0
    self.yellow_img_xypos = np.array([-1, -1])
    self.blue_img_xypos = np.array([-1, -1])
    self.target_yhistory = []  # remember predicted coordinates of the target
    self.target_zhistory = [] 

    # load target template
    self.target_templ = cv2.inRange(cv2.imread('circle1.png', 1), (200, 200, 200), (255, 255, 255))


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
    a = 1/self.pixel_meter_ratio
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


  # Return y-z coordinates of the target (in m) with respect to the yellow joint / base frame
  def detect_target(self, image):
    # detect orange
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([15, 20, 20])
    upper_orange = np.array([25, 255, 255])
    image_orange = cv2.inRange(image_hsv, lower_orange, upper_orange)

    # detect the window to search in

    # find leftmost white pixel's column index
    leftmost_pos = 0
    while not np.any(image_orange[:, leftmost_pos]):
      leftmost_pos += 1

    # find rightmost white pixel's column index
    rightmost_pos = image_orange.shape[1] - 1 
    while not np.any(image_orange[:, rightmost_pos]):
      rightmost_pos -= 1

    # find the uppermost white pixel's row index
    uppermost_pos = 0
    while not np.any(image_orange[uppermost_pos, :]):
      uppermost_pos += 1

    # find the lowermost white pixel's row index
    lowermost_pos = image_orange.shape[0] - 1
    while not np.any(image_orange[lowermost_pos, :]):
      lowermost_pos -= 1
    
    window = image_orange[uppermost_pos:(lowermost_pos + 1), 
                          leftmost_pos:(rightmost_pos + 1)].copy()
    window = np.pad(window, [(self.target_templ.shape[0], ), (self.target_templ.shape[1], )],
                    'constant')

    # do chamfer matching in the window, find position of target relative to the window
    window_dt = cv2.distanceTransform(cv2.bitwise_not(window), cv2.DIST_L2, 0)
    chamfer_sums = np.zeros((window_dt.shape[0] - self.target_templ.shape[0] + 1,
                            window_dt.shape[1] - self.target_templ.shape[1] + 1))
    for row in range(window_dt.shape[0] - self.target_templ.shape[0] + 1):
      for column in range(window_dt.shape[1] - self.target_templ.shape[1] + 1):
        chamfer_sums[row, column] = np.sum(
          window_dt[row:(row + self.target_templ.shape[0]),
          column:(column + self.target_templ.shape[1])] * self.target_templ)
    
    chamfmin_corner_row, chamfmin_corner_col = np.unravel_index(np.argmin(chamfer_sums), 
                          (chamfer_sums.shape[0], chamfer_sums.shape[1]))
    chamfmin_center_row = chamfmin_corner_row + (self.target_templ.shape[0] - 1) // 2
    chamfmin_center_col = chamfmin_corner_col + (self.target_templ.shape[1] - 1) // 2

    # find (y,z) position of target relative to the base frame / yellow joint
    window_corner_row = uppermost_pos - self.target_templ.shape[0]
    window_corner_col = leftmost_pos - self.target_templ.shape[1]
    chamfmin_img_row = window_corner_row + chamfmin_center_row
    chamfmin_img_col = window_corner_col + chamfmin_center_col
    ypredict_p = chamfmin_img_col - self.yellow_img_xypos[0] 
    zpredict_p = self.yellow_img_xypos[1] - chamfmin_img_row
    ypredict_m = ypredict_p/self.pixel_meter_ratio
    zpredict_m = zpredict_p/self.pixel_meter_ratio
    
    
    # check if the estimated position makes sense (deal with occlusion)
    # use the predicted y position as an indicator of tracking the incorrect object
    if len(self.target_yhistory) == 2:

      if abs(ypredict_m - self.target_yhistory[0]) > 1:
        ypredict_m = self.target_yhistory[0] + \
                     (self.target_yhistory[0] - self.target_yhistory[1])
        zpredict_m = self.target_zhistory[0] + \
                     (self.target_zhistory[0] - self.target_zhistory[1])
        ypredict_p = int(np.around(ypredict_m * self.pixel_meter_ratio))
        zpredict_p = int(np.around(zpredict_m * self.pixel_meter_ratio))

      self.target_yhistory.insert(0, ypredict_m)
      self.target_zhistory.insert(0, zpredict_m)
      self.target_yhistory.pop()
      self.target_zhistory.pop()

    else:
      self.target_yhistory.insert(0, ypredict_m)
      self.target_zhistory.insert(0, zpredict_m)
    
   
    #uncomment to show tracking of target
    '''
    chamfmin_img_row = self.yellow_img_xypos[1] - zpredict_p
    chamfmin_img_col = self.yellow_img_xypos[0] + ypredict_p
    image_orange[(chamfmin_img_row-30):(chamfmin_img_row+30), chamfmin_img_col] = 100
    image_orange[chamfmin_img_row, (chamfmin_img_col-30):(chamfmin_img_col+30)] = 100
    im1=cv2.imshow('window1', image_orange)
    cv2.waitKey(1)
    '''


    return np.array([ypredict_m, zpredict_m]) 




  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):

    # first time the node is run callback receives a faulty image from the camera
    if not self.callback_called:
      self.callback_called = True
      return
   

    # Recieve the image
    try:
      cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    

    # Calculate the position of the yellow joint in the image once
    if np.all(self.yellow_img_xypos == np.array([-1, -1])):
      self.yellow_img_xypos = self.detect_yellow1(cv_image1)
    # Calculate the position of the blue joint in the image once
    if np.all(self.blue_img_xypos == np.array([-1, -1])):
      self.blue_img_xypos = self.detect_blue1(cv_image1)

    # Calculate the pixel/meter ratio once
    if self.pixel_meter_ratio == 0:
      self.pixel_meter_ratio = np.linalg.norm(self.yellow_img_xypos - self.blue_img_xypos)/2
      #print(self.pixel_meter_ratio)  
 

    # Detect the target's (y, z) position
    target_yzposition = self.detect_target(cv_image1)



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




    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image1)
    
#-----------------FALUTY Z AXIS(inverse it)---------------

#---------------------------------------------

    # Uncomment to show the feed from the camera
    #im1=cv2.imshow('window1',cv_image1)
    #cv2.waitKey(3)


 
    b = self.detect_joint_positions1(cv_image1)
    self.positions1 = Float64MultiArray()
    self.positions1.data=b
    


    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(cv_image1, "bgr8"))
      self.positions_pub1.publish(self.positions1) 
      #print(np.around(self.positions1.data, 3))

      # publish estimated y-z coordinate of the target     
      self.target_ypredict_pub.publish(target_yzposition[0])
      self.target_zpredict_pub.publish(target_yzposition[1])

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


