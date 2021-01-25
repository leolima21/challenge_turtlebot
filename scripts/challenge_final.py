#!/usr/bin/env python3
import os
import tf
import cv2
import math
import time
import rospy
import apriltag
import numpy as np
import random
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
    
    rospy.init_node('challenge_movebase', anonymous=True) 

    self.move_base_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    self.bridge = CvBridge()
    self.tflistener = tf.TransformListener()

    self.tag_0_status = False
    self.tag_1_status = False
    self.tag_2_status = False

    self.tag_0_p = []
    self.tag_1_p = []
    self.tag_2_p = []

    self.tag_0_go = False
    self.tag_1_go = False
    self.tag_2_go = False

    self.explorer_status = False

    print('[info] ...Done')
  
  def callback(self, data): 
    # Try detect a tag:
    tag_det = self.tag_detect(data)
 
    # If a tag was detected:
    if tag_det >= 0:
      # Save robot point:
      (trans,rot) = self.tflistener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
      
      if tag_det == 0 and self.tag_0_status == False:
        self.tag_0_status = True
        self.tag_0_p = [round(trans[0], 2), 
                        round(trans[1], 2), 
                        round(rot[2], 2),
                        round(rot[3], 2)
                        ]

        print('[info] Tag 0 point:')
        print(self.tag_0_p)

      elif tag_det == 1 and self.tag_1_status == False:
        self.tag_1_status = True
        self.tag_1_p = [round(trans[0], 2), 
                        round(trans[1], 2), 
                        round(rot[2], 2),
                        round(rot[3], 2)
                        ]
        
        print('[info] Tag 1 point:')
        print(self.tag_1_p)

      elif tag_det == 2 and self.tag_2_status == False:
        self.tag_2_status = True
        self.tag_2_p = [round(trans[0], 2), 
                        round(trans[1], 2), 
                        round(rot[2], 2),
                        round(rot[3], 2)
                        ]
        
        print('[info] Tag 2 point:')
        print(self.tag_2_p)

    # If 3 tags was detected:
    if self.tag_0_status == True and self.tag_1_status == True and self.tag_2_status == True:
      if self.explorer_status == False:
        os.system("rosnode kill /explore")
        self.explorer_status = True

      # Move base between the tags:       
      if (tag_det == 0 and self.tag_1_go == False):
        print('[info] ID 0 detected and going to tag 1')
        self.goal_move_base(self.tag_1_p[0], self.tag_1_p[1], self.tag_1_p[2], self.tag_1_p[3])
        self.tag_0_go = False
        self.tag_1_go = True
        self.tag_2_go = False

      elif (tag_det == 1 and self.tag_2_go == False):
        print('[info] ID 1 detected and going to tag 2')
        self.goal_move_base(self.tag_2_p[0], self.tag_2_p[1], self.tag_2_p[2], self.tag_2_p[3])
        self.tag_0_go = False
        self.tag_1_go = False
        self.tag_2_go = True

      elif (tag_det == 2 and self.tag_0_go == False):
        print('[info] ID 2 detected and going to tag 0')
        self.goal_move_base(self.tag_0_p[0], self.tag_0_p[1], self.tag_0_p[2], self.tag_0_p[3])
        self.tag_0_go = True
        self.tag_1_go = False
        self.tag_2_go = False

  def listener(self):
    rospy.Subscriber("picamera/image_raw", Image, self.callback)

    rospy.spin()
  
  def goal_move_base(self, pose_x, pose_y, pose_z, pose_w):
    # setup pub values with x and y positions:
    msg_move_to_goal = PoseStamped()
    msg_move_to_goal.pose.position.x = pose_x 
    msg_move_to_goal.pose.position.y = pose_y
    msg_move_to_goal.pose.orientation.z = pose_z
    msg_move_to_goal.pose.orientation.w = pose_w
    msg_move_to_goal.header.frame_id = 'map'

    self.move_base_pub.publish(msg_move_to_goal)

  def tag_detect(self, data):
    # Convert gray ros img to cv2 img:
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
