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
    # Criacao do objeto de convercao
    self.bridge = CvBridge()
    # create a camera node:
    rospy.init_node('tag_detector', anonymous=True)
    # publisher objects:
    self.tag_id_pub = rospy.Publisher('challenge/tag_id', Int32, queue_size=10)
    # self.tag_image_pub = rospy.Publisher('challenge/tag_image', Image, queue_size=10)

  def callback(self, data):    
    # Converter a imagem ros para imagem cv2
    cv2_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    cv2_frame_grey = cv2.cvtColor(cv2_frame, cv2.COLOR_BGR2GRAY)

    # Tag detection
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(cv2_frame_grey)
    
    # for r in results:
    #   (cX, cY) = (int(r.center[0]), int(r.center[1]))
    #   cv2.circle(cv2_frame, (cX, cY), 5, (0,0,255), -1)

    if len(results) > 0:
      self.tag_id_pub.publish(results[0].tag_id)

    print('[info] buscando por tags...')
    # self.tag_image_pub.publish(Image)

  def listener(self):
    # Inscricao no topico e definicao da callback como funcao a ser executada
    rospy.Subscriber("picamera/image_raw", Image, self.callback)

    # Mantem o python funcionando apos o encerramendo do node
    rospy.spin()

# Funcao main
if __name__ == '__main__':
  try:
    challenge = Challenge()
    challenge.listener()
  except rospy.ROSInterruptException:
    pass	
