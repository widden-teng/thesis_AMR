#!/usr/bin/env python
import roslib
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np
from geometry_msgs.msg import Point, PoseArray, Pose
import pyrealsense2
from std_msgs.msg import Bool
import time






class limit_with_depth(object):
  def __init__(self):

    #### For sync RGBD information
    self.bridge = CvBridge()
    self.image_rgb = None
    self.image_depth = None
    self.camera_info = None
    # 這邊的sub並沒有宣告callback function
    self.image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    self.camera_info_sub = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)
    self.depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    #用來判斷三個msg是否都有傳到topic(同步)  
    time_sync = message_filters.TimeSynchronizer([self.image_sub, self.camera_info_sub, self.depth_sub], 10)
    time_sync.registerCallback(self.image_cb)   
    self.IMG_SIZE = (640,480)

    # for fps
    self.pTime = 0
    self.cTime = 0

    #### Pub

    self.xyz_vector_pub = rospy.Publisher("handpose_cameralink",PoseArray, queue_size=1)
    # self.mediapipe_img_pub = rospy.Publisher("image_topic_2", Image, queue_size = 100)

    self.stop_update_target_pub = rospy.Publisher("stop_update_target_flag",Bool, queue_size=1)


  def image_cb( self, image_msg, info_msg, depth_msg):
    try:
      #cv2通常吃bgr8
      self.image_rgb = self.bridge.imgmsg_to_cv2(image_msg, "bgr8") 
      self.camera_info = info_msg
      self.image_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough").copy()
      #當沒吃到深度時, 深度為0(可能過熱或那個點沒吃到ex:太遠太小)
      self.image_depth[np.isnan(self.image_depth)] = 0.
      self.image_time = image_msg.header.stamp


      depth_mask = self.image_depth > 350
      zero_mask = self.image_depth == 0

      self.image_rgb[depth_mask] = 0
      self.image_rgb[zero_mask] = 0
    except CvBridgeError as e:
      print(e)


    # print(self.image_depth.shape)
    print(self.image_depth[342, 250])



    # 顯示影像
    cv2.imshow('handdetect', self.image_rgb)
    cv2.waitKey(3)
    
    
    
    # cv2.imshow("video stream", cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB))
    # cv2.waitKey(3)

    # 將影像傳到topic上
    # try:
    #   self.mediapipe_img_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)
  



        

def main():
  rospy.init_node('demo_handpose', anonymous=True)
  mew_rgb_frame = limit_with_depth()
  rospy.spin()

if __name__ == '__main__':
  main()