#!/usr/bin/env python
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
import time

################################################################
USELEFTHAND = True



#含校正工具(但僅有上方的camera)
EFF_TOOL_DISTANCE = 0.27 #含校正工具
# HANDOVER_DIS = EFF_TOOL_DISTANCE + 0.05
HANDOVER_DIS = 0 # 原本是用0, 在輸入位置控制時強至x剪 n cm 


DISPLACEMENT_LOW_THREAD = 0.035
DISPLACEMENT_UP_THREAD = 0.2
LIMIT_LAST_TARGET = True
LAST_TARGET_DIS = HANDOVER_DIS + 0.05 #0.35

################################################################
modify_hand = -1 if USELEFTHAND else 1
# 左右手相反
target_hand = "Right" if USELEFTHAND else "Left"


mp_drawing = mp.solutions.drawing_utils          # mediapipe 繪圖方法
mp_drawing_styles = mp.solutions.drawing_styles  # mediapipe 繪圖樣式
mp_hands = mp.solutions.hands                    # mediapipe 偵測手掌方法


class handpose_mediapipe(object):
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

    self.xyz_vector_pub = rospy.Publisher("/camera/handpose_cameralink",PoseArray, queue_size=1)
    self.mediapipe_img_pub = rospy.Publisher("/camera/annotated_image", Image, queue_size = 100)

    self.stop_update_target_pub = rospy.Publisher("stop_update_target_flag",Bool, queue_size=1)

    # for fps
    self.pTime = 0
    self.cTime = 0

    

    # #### Sub


    ### Class variable
    self.depth0 = 0
    self.depth5 = 0
    self.depth13 = 0
    self.u0 = self.v0 = 0
    self.u5 = self.v5 = 0
    self.u9 = self.v9 = 0
    self.u13 = self.v13 = 0
    self.xyz_vector = PoseArray()
    self.sub_flag = False
    self.rate = rospy.Rate(10)
    self.pose_tempx = Pose()
    self.pose_tempy = Pose()
    self.pose_tempz = Pose()
    self.pose_temppt = Pose()
    self.lest_point = Point()
    self.lest_point_array = np.array([0, 0, 0])

    #for testing
    self.testint = 0

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
    except CvBridgeError as e:
      print(e)
    with mp_hands.Hands(model_complexity=0, max_num_hands=1,min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:

      img = self.image_rgb.copy()      
      w = self.IMG_SIZE[0]        # 取得畫面寬度
      h = self.IMG_SIZE[1]       # 取得畫面高度
      img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)   # 將 BGR 轉換成 RGB
      results = hands.process(img_rgb)                 # 偵測手掌
      if results.multi_hand_landmarks:

        # 這邊左右相反 !!!!!!!!!!!!!!!!!!
        hand_side_classification_list = results.multi_handedness[0]
        hand_side = hand_side_classification_list.classification[0].label
        if (hand_side == target_hand):
          # print("in")

          for hand_landmarks in results.multi_hand_landmarks:
            # 將節點和骨架繪製到影像中
            mp_drawing.draw_landmarks(
              img,
              hand_landmarks,
              mp_hands.HAND_CONNECTIONS,
              mp_drawing_styles.get_default_hand_landmarks_style(),
              mp_drawing_styles.get_default_hand_connections_style())
            self.u0 = int(hand_landmarks.landmark[0].x * w)   # 取得palm x 座標
            self.v0 = int(hand_landmarks.landmark[0].y * h)   # 取得palm y 座標

            self.u5 = int(hand_landmarks.landmark[5].x * w)   # 取得palm x 座標
            self.v5 = int(hand_landmarks.landmark[5].y * h)   # 取得palm y 座標

            self.u9 = int(hand_landmarks.landmark[9].x * w)   # 取得palm x 座標
            self.v9 = int(hand_landmarks.landmark[9].y * h)   # 取得palm y 座標

            self.u13 = int(hand_landmarks.landmark[13].x * w)   # 取得palm x 座標
            self.v13 = int(hand_landmarks.landmark[13].y * h)   # 取得palm y 座標


            # self.u = int(hand_landmarks.landmark[0].x * w)   # 取得palm x 座標
            # self.v = int(hand_landmarks.landmark[0].y * h)   # 取得palm y 座標
          
          
          
          try: 

            self.depth0 = self.image_depth[self.v0, self.u0]
            self.depth5 = self.image_depth[self.v5, self.u5]
            self.depth9 = self.image_depth[self.v9, self.u9]
            self.depth13 = self.image_depth[self.v13, self.u13]
            # print("u0 , v0 pose:", self.u0, self.v0) 
            # print("depth0 :", self.depth0)
            # print("u5 , v5 pose:", self.u5, self.v5) 
            # print("depth5 :", self.depth5)
            # print("u13 , v13 pose:", self.u13, self.v13) 
            # print("depth13 :", self.depth13)
            if ((not LIMIT_LAST_TARGET) or (self.depth0/1000 > LAST_TARGET_DIS)):
              # print("not in LAST_TARGET_DIS")
              if (self.depth0 != 0 or self.depth5 != 0 or self.depth13 != 0):
                # 這邊轉成camera link 的 frame
                pinhole0 = self.pinhole_trans_def(Point(self.u0, self.v0, self.depth0))
                pinhole5 = self.pinhole_trans_def(Point(self.u5, self.v5, self.depth5)) 
                pinhole9 = self.pinhole_trans_def(Point(self.u9, self.v9, self.depth9))
                pinhole13 = self.pinhole_trans_def(Point(self.u13, self.v13, self.depth13))
                
                

                vector0_9 = pinhole9 - pinhole0
                vector0_5 = pinhole5 - pinhole0
                vector0_13 = pinhole13 - pinhole0

                
                handover_vector_z = np.cross(vector0_5, vector0_13) / np.linalg.norm(np.cross(vector0_5, vector0_13))
                handover_vector_y = vector0_9 / np.linalg.norm(vector0_9)
                handover_vector_x = np.cross(handover_vector_y, handover_vector_z) / np.linalg.norm(np.cross(handover_vector_y, handover_vector_z))

                handover_point = pinhole0 + modify_hand * HANDOVER_DIS * handover_vector_z
                # self.pinhole_pub.publish(Point(handover_point[0], handover_point[1], handover_point[2]))

                self.pose_tempx.position.x = handover_vector_x[0]
                self.pose_tempx.position.y = handover_vector_x[1]
                self.pose_tempx.position.z = handover_vector_x[2]
                self.xyz_vector.poses.append(self.pose_tempx)


                self.pose_tempy.position.x = handover_vector_y[0]
                self.pose_tempy.position.y = handover_vector_y[1]
                self.pose_tempy.position.z = handover_vector_y[2]
                self.xyz_vector.poses.append(self.pose_tempy)

                self.pose_tempz.position.x = handover_vector_z[0]
                self.pose_tempz.position.y = handover_vector_z[1]
                self.pose_tempz.position.z = handover_vector_z[2]
                self.xyz_vector.poses.append(self.pose_tempz)

                self.pose_temppt.position.x = handover_point[0]
                self.pose_temppt.position.y = handover_point[1]
                self.pose_temppt.position.z = handover_point[2]
                self.xyz_vector.poses.append(self.pose_temppt)
                  

                displacement = 0
                displacement = np.linalg.norm(self.lest_point_array - np.array(handover_point))

                # if (displacement > DISPLACEMENT_LOW_THREAD and displacement > DISPLACEMENT_UP_THREAD):
                #   self.xyz_vector_pub.publish(self.xyz_vector)
                # self.lest_point_array = np.array(handover_point)
                # self.xyz_vector.poses.clear()

                ##### for no limit###################
                self.xyz_vector_pub.publish(self.xyz_vector)
                self.xyz_vector.poses.clear()
                ####################################
            # else:
              # print("!!!!!!!!!!!!!!!!")

              ####test#####
              self.sub_flag = False
              # while not self.sub_flag:
              #   # .get_num_connections()被用來檢查發佈者 pub 是否已經有與其相連的訂閱者
              #   # 用於確保在發佈訊息之前至少有一個訂閱者
              #   if self.xyz_vector_pub.get_num_connections()>0:
              #       self.xyz_vector_pub.publish(self.xyz_vector)
              #       self.sub_flag = True
              #   else:
              #       self.rate.sleep()
              ####test#####
              
              


              # print("pub handover pose :", handover_point)
              # print("handover_vector_x: ", handover_vector_x)
              # print("handover_vector_y: ", handover_vector_y)
              # print("handover_vector_z: ", handover_vector_z)


              

          except:
            print("depth err")
          self.depth0 = self.depth5 = self.depth13 = 0
          self.u0 = self.v0 = self.u5 = self.v5 = self.u9 = self.v9 = self.u13 = self.v13 = 0

      # #fps
      # self.cTime =time.time()
      # fps = 1/(self.cTime-self.pTime)
      # self.pTime = self.cTime
      # cv2.putText(img, f"FPS:{int(fps)}", (30, 50), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1)
      # 顯示影像
      # cv2.imshow('handdetect', img)
      # cv2.waitKey(3)
      # cv2.imshow("video stream", cv2.cvtColor(img_rgb, cv2.COLOR_BGR2RGB))
      # cv2.waitKey(3)

      # 將影像傳到topic上
      try:
        self.mediapipe_img_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
      except CvBridgeError as e:
        print(e)
  


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
    
      
  # def pinhole_trans(self, point):
  #   _intrinsics = pyrealsense2.intrinsics()
  #   _intrinsics.width = self.camera_info.width
  #   _intrinsics.height = self.camera_info.height
  #   _intrinsics.ppx = self.camera_info.K[2]
  #   _intrinsics.ppy = self.camera_info.K[5]
  #   _intrinsics.fx = self.camera_info.K[0]
  #   _intrinsics.fy = self.camera_info.K[4]
  #   #_intrinsics.model = cameraInfo.distortion_model
  #   _intrinsics.model  = pyrealsense2.distortion.none
  #   # 用來做径向畸变系数和切向畸变系数进行校正
  #   _intrinsics.coeffs = [i for i in self.camera_info.D]
  #   result = pyrealsense2.rs2_deproject_pixel_to_point(_intrinsics, [point.x, point.y], point.z)
  #   # 將XYZ單位變成m
  #   result = np.array(result) / 1000 
  #   self.pinhole_pub.publish(Point(result[2], -result[0], -result[1]))
    
    

        

def main():
  rospy.init_node('handpose_evaluation', anonymous=True)
  hand_pose = handpose_mediapipe()
  rospy.spin()

if __name__ == '__main__':
  main()