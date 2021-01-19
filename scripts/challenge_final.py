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
    print('[info] initializing vars...')
    
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

    print('[info] variaveis iniciadas')
  
  def callback(self, data): 
    tag_det = self.tag_detect(data)

    if tag_det >= 0:
      (trans,rot) = self.tflistener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
      
      if tag_det == 0 and self.tag_0_status == False:
        self.tag_0_status = True
        self.tag_0_p = [round(trans[0], 2), 
                        round(trans[1], 2), 
                        round(rot[2], 2),
                        round(rot[3], 2)
                        ]
        # self.goal_move_base(0, 0, -1, 1)
        print(self.tag_0_p)

      elif tag_det == 1 and self.tag_1_status == False:
        self.tag_1_status = True
        self.tag_1_p = [round(trans[0], 2), 
                        round(trans[1], 2), 
                        round(rot[2], 2),
                        round(rot[3], 2)
                        ]
        # self.goal_move_base(0, 0, -1, 1)
        print(self.tag_1_p)

      elif tag_det == 2 and self.tag_2_status == False:
        self.tag_2_status = True
        self.tag_2_p = [round(trans[0], 2), 
                        round(trans[1], 2), 
                        round(rot[2], 2),
                        round(rot[3], 2)
                        ]
        # self.goal_move_base(0, 0, -1, 1)
        print(self.tag_2_p)

    if self.tag_0_status == True and self.tag_1_status == True and self.tag_2_status == True:
      # rosnode kill explorer--------------------      
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
    # Inscricao no topico e definicao da callback como funcao a ser executada
    rospy.Subscriber("picamera/image_raw", Image, self.callback)

    # Mantem o python funcionando apos o encerramendo do node
    rospy.spin()
  
  def goal_move_base(self, pose_x, pose_y, pose_z, pose_w):
    # setup pub values with x and y positions
    msg_move_to_goal = PoseStamped()
    msg_move_to_goal.pose.position.x = pose_x 
    msg_move_to_goal.pose.position.y = pose_y
    msg_move_to_goal.pose.orientation.z = pose_z
    msg_move_to_goal.pose.orientation.w = pose_w
    msg_move_to_goal.header.frame_id = 'map'

    self.move_base_pub.publish(msg_move_to_goal)

  def cmd_vel_pub(self, linear, angular):
    vel_msg = Twist()
    vel_msg.linear.x = linear
    vel_msg.angular.z = angular
    self.velocity_pub.publish(vel_msg)

  def tag_detect(self, data):
    # Converter a imagem ros para imagem cv2
    cv2_frame = self.bridge.imgmsg_to_cv2(data, "mono8")
    # cv2_frame_grey = cv2.cvtColor(cv2_frame, cv2.COLOR_BGR2GRAY)

    # Tag detection
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(cv2_frame)
    
    # for r in results:
    #   (cX, cY) = (int(r.center[0]), int(r.center[1]))
    #   cv2.circle(cv2_frame, (cX, cY), 5, (0,0,255), -1)

    if len(results) > 0:
      return results[0].tag_id
      # print('[info] tag detectada')
    else:
      return -1
      print('[info] buscando por tags...')

  def print_info(self, stat0, stat1, stat2):
    print('----------------------')
    print('TAG 0: ' + str(stat0))
    print('TAG 1: ' + str(stat1))
    print('TAG 2: ' + str(stat2))

# Funcao main
if __name__ == '__main__':
  try:
    challenge = Challenge()
    challenge.listener()
  except rospy.ROSInterruptException:
    pass	
