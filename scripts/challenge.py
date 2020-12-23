#!/usr/bin/env python3
import rospy
import cv2
import sys
import time
import apriltag
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Challenge:
  def __init__(self):
    # self.tag_image_pub = rospy.Publisher('challenge/tag_image', Image, queue_size=10)

  def callback(self, data):    
    

  def listener(self):
    # Inscricao no topico e definicao da callback como funcao a ser executada
    rospy.Subscriber("challenge/tag_id", Int32, self.callback)

    # Mantem o python funcionando apos o encerramendo do node
    rospy.spin()

# Funcao main
if __name__ == '__main__':
  try:
    challenge = Challenge()
    challenge.listener()
  except rospy.ROSInterruptException:
    pass	
