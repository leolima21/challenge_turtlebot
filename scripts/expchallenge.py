#!/usr/bin/env python3
import os
import cv2
import math
import time
import rospy
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

class Challenge:
  def __init__(self):
    self.move_base_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=0)
    rospy.init_node('challenge_movebase', anonymous=True)  

    self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=0)

    self.time_var = 300
    
  def callback(self, data):    
    if self.time_var >= 300:
      self.time_var = 0
      rand_x = random.randint(-1,1)
      rand_y = random.randint(-1,1)
      rand_z = random.randint(-1,1)
      rand_w = 0.67

      self.goal_move_base(rand_x, rand_y, rand_z, rand_w)

    self.time_var += 1
    # print(self.time_var)

  def callback_2(self, data):
    print('[info] tag ' + str(data.data) + ' detectada')
    self.goal_move_base(0, 0, 0, 0)
    # self.cmd_vel_pub(0,0)
    self.time_var = 0
    
  def listener(self):
    # Inscricao no topico e definicao da callback como funcao a ser executada
    rospy.Subscriber("challenge/tag_id", Int32, self.callback_2)
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
    msg_move_to_goal.header.frame_id = 'base_footprint'
    # msg_move_to_goal.header.frame_id = 'base_footprint'

    self.move_base_pub.publish(msg_move_to_goal)
    # sleep(20)
    # add aqui a espera pelo resutado do move base

  def cmd_vel_pub(self, linear, angular):
    vel_msg = Twist()
    vel_msg.linear.x = linear
    vel_msg.angular.z = angular
    self.velocity_publisher.publish(vel_msg)

# Funcao main
if __name__ == '__main__':
  try:
    challenge = Challenge()
    challenge.listener()
  except rospy.ROSInterruptException:
    pass	
