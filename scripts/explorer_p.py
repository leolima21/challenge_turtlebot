#!/usr/bin/env python3
import os
import tf
import cv2
import math
import time
import rospy
import apriltag
import numpy as np
from random import *
from time import sleep
from std_msgs.msg import Int32
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from actionlib_msgs.msg import GoalID 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseActionGoal

import roslib
# roslib.load_manifest('learning_tf')

class Challenge:
  def __init__(self):
    print('[info] Starting vars...')
    
    rospy.init_node('explore')

    self.move_base_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    self.bridge = CvBridge()

    self.time_var = 150

    self.tag_0_status = False
    self.tag_1_status = False
    self.tag_2_status = False

    print('[info] ...Done')
  
  def callback(self, data): 
    # tag_det = self.tag_detect(data)
    
    # if (self.time_var >= 300 and tag_det < 0):
    if (self.time_var >= 150):  
      self.time_var = 0
      f_rand_x = round(uniform(1,2), 2)
      f_rand_y = round(uniform(-1,1), 2)
      # f_rand_z = round(uniform(-1,1), 2)

      # rand_x = random.randint(0,1)
      # rand_y = random.randint(-1,1)
      rand_z = randint(-1,1)

      rand_w = 0.67

      self.goal_move_base(f_rand_x, f_rand_y, rand_z, rand_w)
      rospy.loginfo('point')     
    # elif (tag_det == 0 and self.tag_0_status == False):
    #   self.tag_0_status = True
    #   self.time_var = 0
    #   print('det 0')
    #   self.goal_move_base(0, -1, 0, 1) #(-2)
    # elif (tag_det == 1 and self.tag_1_status == False):
    #   self.tag_1_status = True
    #   self.time_var = 0
    #   print('det 1')
    #   self.goal_move_base(0, 1, 0, 1) #(1)
    # elif (tag_det == 2 and self.tag_2_status == False):
    #   self.tag_2_status = True
    #   self.time_var = 0
    #   print('det 2')
    #   self.goal_move_base(-1, 1, -1, 1)
          
    self.time_var += 1
    
  def listener(self):
    rospy.Subscriber("picamera/image_raw", Image, self.callback)

    rospy.spin()
  
  def goal_move_base(self, pose_x, pose_y, pose_z, pose_w):
    # setup pub values with x and y positions
    msg_move_to_goal = PoseStamped()
    msg_move_to_goal.pose.position.x = pose_x 
    msg_move_to_goal.pose.position.y = pose_y
    msg_move_to_goal.pose.orientation.z = pose_z
    msg_move_to_goal.pose.orientation.w = pose_w
    msg_move_to_goal.header.frame_id = 'base_footprint'

    self.move_base_pub.publish(msg_move_to_goal)

  def tag_detect(self, data):
    # Convert ros gray img to cv2:
    cv2_frame = self.bridge.imgmsg_to_cv2(data, "mono8")

    # Tag detection:
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(cv2_frame)

    if len(results) > 0:
      return results[0].tag_id
    else:
      return -1

# Main function:
if __name__ == '__main__':
  try:
    challenge = Challenge()
    challenge.listener()
  except rospy.ROSInterruptException:
    pass	
