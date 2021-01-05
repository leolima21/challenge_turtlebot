#!/usr/bin/env python3
import os
import cv2
import math
import time
import rospy
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from actionlib_msgs.msg import GoalID 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseActionGoal

class Challenge:
  def __init__(self):
    self.move_base_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    rospy.init_node('challenge_movebase', anonymous=True)

  def callback(self, data):    
    if (data.data == 0):
      print('[info] ID 0 detected')
      self.goal_move_base(3.14, -3.48, 0.77, 0.63)
    elif (data.data == 1):
      print('[info] ID 1 detected')
      self.goal_move_base(5.79, -2.12, 0.17, 0.98)
    elif (data.data == 2):
      print('[info] ID 2 detected')
      self.goal_move_base(1.07, -2.17, -0.99, 0.06)
    else:
      print('[info] Unknow error ou unknow tag detected')

  def listener(self):
    # Inscricao no topico e definicao da callback como funcao a ser executada
    rospy.Subscriber("challenge/tag_id", Int32, self.callback)

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
    # msg_move_to_goal.header.frame_id = 'base_footprint'

    self.move_base_pub.publish(msg_move_to_goal)

# Funcao main
if __name__ == '__main__':
  try:
    challenge = Challenge()
    challenge.listener()
  except rospy.ROSInterruptException:
    pass	
