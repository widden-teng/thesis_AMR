#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import mediapipe as mp
import message_filters
import numpy as np
from geometry_msgs.msg import Point, PoseArray, Pose
import pyrealsense2
from std_msgs.msg import Bool



class fix_img(object):
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

    #### Pub

    self.xyz_vector_pub = rospy.Publisher("handpose_cameralink",PoseArray, queue_size=1)
    # self.mediapipe_img_pub = rospy.Publisher("image_topic_2", Image, queue_size = 100)

    self.stop_update_target_pub = rospy.Publisher("stop_update_target_flag",Bool, queue_size=1)
    


  def image_cb( self, image_msg, info_msg, depth_msg):
    try:
      #cv2通常吃bgr8
      self.image_rgb = self.bridge.imgmsg_to_cv2(image_msg, "bgr8") 
       #這是學長加的(原本沒用途)
      # self.camera_info = np.array(info_msg.P).reshape(3, 4)
      self.camera_info = info_msg
      self.image_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough").copy()
      #當沒吃到深度時, 深度為0(可能過熱或那個點沒吃到ex:太遠太小)
      self.image_depth[np.isnan(self.image_depth)] = 0.
      self.image_time = image_msg.header.stamp

      # condition1 = self.image_depth > 2500
      # condition2 = self.image_depth == 0

      # # 將兩個布林條件合併成一個
      # combined_condition = np.logical_or(condition1, condition2)
      # rows, cols = np.where(combined_condition)
      # self.image_rgb[rows, cols] = [0, 0, 0]
      # # print(self.image_depth[240, 320])

    

    except:
      print("depth err")
    self.depth0 = self.depth5 = self.depth13 = 0
    self.u0 = self.v0 = self.u5 = self.v5 = self.u9 = self.v9 = self.u13 = self.v13 = 0


    # 顯示影像
    cv2.imshow('handdetect', self.image_rgb)
    cv2.waitKey(3)
  


  def pinhole_trans_def(self, point):
    _intrinsics = pyrealsense2.intrinsics()
    _intrinsics.width = self.camera_info.width
    _intrinsics.height = self.camera_info.height
    _intrinsics.ppx = self.camera_info.K[2]
    _intrinsics.ppy = self.camera_info.K[5]
    _intrinsics.fx = self.camera_info.K[0]
    _intrinsics.fy = self.camera_info.K[4]
    #_intrinsics.model = cameraInfo.distortion_model
    _intrinsics.model  = pyrealsense2.distortion.none
    # 用來做径向畸变系数和切向畸变系数进行校正
    _intrinsics.coeffs = [i for i in self.camera_info.D]
    result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [point.x, point.y], point.z)
    # 將XYZ單位變成m
    result = np.array(result) / 1000
    return np.array([result[2], -result[0], -result[1]]) 
    

        

def main():
  rospy.init_node('img_fix_by_depth', anonymous=True)
  fix_img_temp = fix_img()
  rospy.spin()

if __name__ == '__main__':
  main()