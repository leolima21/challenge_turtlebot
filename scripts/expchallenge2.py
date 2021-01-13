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
    print('[info] iniciando variaveis...')
    
    rospy.init_node('challenge_movebase', anonymous=True) 

    # testar o queue_size com None, 1 e 10
    self.move_base_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=None)
    self.velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=None)

    self.time_var = 300

    self.tag_0_status = False
    self.tag_1_status = False
    self.tag_2_status = False

    self.clear = lambda: os.system('cls')

    print('[info] variaveis iniciadas')
    
  def callback(self, data): 
    if (data.data >= 0):
      self.cmd_vel_pub(0,0)
      # print('[info] tag ' + str(data.data) + ' detectada')
    else:
      if (self.time_var >= 300):
        self.time_var = 0
        rand_x = random.randint(-1,1)
        rand_y = random.randint(-1,1)
        rand_z = random.randint(-1,1)
        rand_w = 0.67
        self.goal_move_base(rand_x, rand_y, rand_z, rand_w) 
          
    self.time_var += 1

    self.print_info(self.tag_0_status, self.tag_1_status, self.tag_2_status)
    
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
    msg_move_to_goal.header.frame_id = 'base_footprint'

    self.move_base_pub.publish(msg_move_to_goal)

  def cmd_vel_pub(self, linear, angular):
    vel_msg = Twist()
    vel_msg.linear.x = linear
    vel_msg.angular.z = angular
    self.velocity_pub.publish(vel_msg)

  def print_info(self, stat0, stat1, stat2):
    print('----------------------')
    print('TAG 0: ' + stat0)
    print('TAG 1: ' + stat1)
    print('TAG 2: ' + stat2)

    self.clear()

# Funcao main
if __name__ == '__main__':
  try:
    challenge = Challenge()
    challenge.listener()
  except rospy.ROSInterruptException:
    pass	
