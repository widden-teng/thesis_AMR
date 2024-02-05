#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from geometry_msgs.msg import Point, PoseArray, Pose
from std_msgs.msg import Bool, String
import pyrealsense2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import time
import cv2
import math

############### global variables ################################################

### approach variables 
LAST_TARGET_DIS = 0.40  #0.25
HANDOVER_DIS = 0.23
DISPLACEMENT_LOW_THREAD = 0.035
DISPLACEMENT_UP_THREAD = 0.2

###gesture ###
base_options_gesture = python.BaseOptions(model_asset_path='/home/user/cuda11.2cudnn8/docker_ws/src/test_pkg/scripts/weight/gesture_recognizer_1030poch10.task')
### gesture visulization
MARGIN = 10  # pixels
FONT_SIZE = 1
FONT_THICKNESS = 1
HANDEDNESS_TEXT_COLOR = (88, 205, 54) # vibrant green
VisionRunningMode = mp.tasks.vision.RunningMode
GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult


###landmarks ###
base_options_landmark = python.BaseOptions(model_asset_path='/home/user/cuda11.2cudnn8/docker_ws/src/test_pkg/scripts/weight/hand_landmarker.task',delegate=python.BaseOptions.Delegate.GPU)
### hand landmark visulization
mp_drawing = mp.solutions.drawing_utils          # mediapipe 繪圖方法
custom_handLmsStyle = mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2)
custom_handConStyle = mp_drawing.DrawingSpec(color=(128, 128, 128), thickness=3)

####################################################################################


class handpose_mediapipe(object):
  def __init__(self):

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
    self.pose_temppt0 = Pose()
    self.last_point_array = np.array([0, 0, 0])
    self.handedness_factor = 0
    self.handedness_new = "Right"

    #### For sync RGBD information
    self.bridge = CvBridge()
    self.image_rgb = None
    self.image_depth = None
    self.camera_info = None

    ### camera 相關
    # 這邊的sub並沒有宣告callback function
    self.image_sub = message_filters.Subscriber('/side_camera/color/image_raw', Image)
    self.camera_info_sub = message_filters.Subscriber('/side_camera/color/camera_info', CameraInfo)
    self.depth_sub = message_filters.Subscriber('/side_camera/aligned_depth_to_color/image_raw', Image)
    #用來判斷三個msg是否都有傳到topic(同步)  
    time_sync = message_filters.TimeSynchronizer([self.image_sub, self.camera_info_sub, self.depth_sub], 10)
    time_sync.registerCallback(self.image_cb)   
    self.IMG_SIZE = (640,480)

    ### Sub
    rospy.Subscriber("/start_gesture", Bool, self.startGestureCB)

    #### Pub
    self.xyz_vector_pub = rospy.Publisher("/side_camera/handpose_cameralink",PoseArray, queue_size=1)
    self.mediapipe_img_pub = rospy.Publisher("/side_camera/annotated_image", Image, queue_size = 100)
    #用來UKF更換發tf的通知(看手掌是否距離camera過近)
    self.stop_update_target_pub = rospy.Publisher("/side_camera/stop_update_target_flag",Bool, queue_size=1)
    self.gesture_pub = rospy.Publisher('/side_camera/gesture_topic', String, queue_size=1)

    ### for stop update target flag 
    ### only when the point0(palm) > LAST_TARGET_DIS , it will be False(switch who board tf in ukf), and will pub the point, otherwise it wont 
    self.stop_update_target_bool_msg = Bool()
    self.stop_update_target_bool_msg.data = True  


    # for fps
    self.pTime = 0
    self.cTime = 0

    ### landmarks
    self.options_landmark = vision.HandLandmarkerOptions(base_options=base_options_landmark,num_hands=1)
    self.detector_landmark = vision.HandLandmarker.create_from_options(self.options_landmark)    # 這一行會出現cpu.....
    ### recognition
    options = vision.GestureRecognizerOptions(base_options=base_options_gesture, running_mode=VisionRunningMode.LIVE_STREAM,result_callback=self.get_pub_result)
    self.recognizer = vision.GestureRecognizer.create_from_options(options)
    self.gesture_name = None
    self.start_gesture_flag = False
    self.finish_one_gesture_recog_flag = False


  def startGestureCB(self,msg):
    if msg.data:
      self.start_gesture_flag = True
    else:
      self.start_gesture_flag = False
      

  def get_pub_result(self,result: GestureRecognizerResult, output_image: mp.Image, timestamp_ms: int):
    for gesture in result.gestures:
      for category in gesture:
        # print("recognition :",category.category_name)
        self.gesture_name = category.category_name
        if self.start_gesture_flag:
          self.gesture_pub.publish(self.gesture_name)
          self.finish_one_gesture_recog_flag = True
        break
  
  def draw_and_pub_landmark(self, rgb_image, detection_result):
    hand_landmarks_list = detection_result.hand_landmarks
    handedness_list = detection_result.handedness
    annotated_image = np.copy(rgb_image)

    #這邊因為num_hands 設為1, 所以只執行一次
    for idx in range(len(hand_landmarks_list)):
      hand_landmarks = hand_landmarks_list[idx]
      handedness = handedness_list[idx]
      if(handedness[0].category_name == "Left"):
        self.handedness_new = "Left"
        self.handedness_factor = -1  
      elif(handedness[0].category_name == "Right"):
        self.handedness_new = "Right"
        self.handedness_factor = 1

      ###################### Draw the hand landmarks ######################
      hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
      hand_landmarks_proto.landmark.extend([
        landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
      ])
      solutions.drawing_utils.draw_landmarks(
        annotated_image,
        hand_landmarks_proto,
        solutions.hands.HAND_CONNECTIONS,
        custom_handLmsStyle,
        custom_handConStyle)

      # Get the top left corner of the detected hand's bounding box.
      height, width, _ = annotated_image.shape
      x_coordinates = [landmark.x * width for landmark in hand_landmarks]
      y_coordinates = [landmark.y * height for landmark in hand_landmarks]
      text_x = int(min(x_coordinates) )
      text_y = int(min(y_coordinates) ) - MARGIN

      # Draw handedness (left or right hand) on the image.
      # 顯示在手的上面
      # cv2.putText(annotated_image, f"{self.gesture_name}",
      #             (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX,
      #             FONT_SIZE, HANDEDNESS_TEXT_COLOR, FONT_THICKNESS, cv2.LINE_AA)
      # cv2.putText(annotated_image, f"Gesture:{self.handedness_new}", (30, 90), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1)
      
      ######################################################################
      
      ################### pub handpose imformation######################
      self.u0 = int(x_coordinates[0])   # 取得palm x 座標
      self.v0 = int(y_coordinates[0])   # 取得palm y 座標

      self.u5 = int(x_coordinates[5])   # 取得palm x 座標
      self.v5 = int(y_coordinates[5])   # 取得palm y 座標

      self.u9 = int(x_coordinates[9])   # 取得palm x 座標
      self.v9 = int(y_coordinates[9])   # 取得palm y 座標

      self.u13 = int(x_coordinates[13])   # 取得palm x 座標
      self.v13 = int(y_coordinates[13])   # 取得palm y 座標
      # print("new u0,v0 is : {},{}".format(self.u0, self.v0))

      try: 
        #這邊u, v 相反
        self.depth0 = self.image_depth[self.v0, self.u0]
        self.depth5 = self.image_depth[self.v5, self.u5]
        self.depth9 = self.image_depth[self.v9, self.u9]
        self.depth13 = self.image_depth[self.v13, self.u13]
        if ((self.depth0/1000 > LAST_TARGET_DIS)):
          # 代表可以用此camera的資料
          self.stop_update_target_bool_msg = False

          # print("Side depth is :", self.depth0/1000)
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

            handover_point = pinhole0 + self.handedness_factor * HANDOVER_DIS * handover_vector_z
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

            # 這邊 pose_temppt0 用一樣的(用來發手掌的點, 測試用)
            self.pose_temppt0.position.x = pinhole0[0]
            self.pose_temppt0.position.y = pinhole0[1]
            self.pose_temppt0.position.z = pinhole0[2]
            self.xyz_vector.poses.append(self.pose_temppt0)

            #time
            self.xyz_vector.header.stamp = rospy.Time.now() 
                     
            ### 用來避免震盪與突然跑太遠(誤判)#######
            # displacement = 0
            # displacement = np.linalg.norm(self.last_point_array - np.array(handover_point))
            # if (displacement > DISPLACEMENT_LOW_THREAD and displacement > DISPLACEMENT_UP_THREAD):
            #   self.xyz_vector_pub.publish(self.xyz_vector)
            # self.last_point_array = np.array(handover_point)
            # self.xyz_vector.poses.clear()
            #####################################

            ### avoid x, y, z axis vectors being 0  ###
            for pose in self.xyz_vector.poses:
              if any(math.isnan(value) for value in [pose.position.x, pose.position.y, pose.position.z]):
                print("one of xyz vector is nan")
                raise
            ###################################

            self.xyz_vector_pub.publish(self.xyz_vector)
            self.xyz_vector.poses.clear()   
                                   
            ####test#####
            # self.sub_flag = False
            # while not self.sub_flag:
            #   # .get_num_connections()被用來檢查發佈者 pub 是否已經有與其相連的訂閱者
            #   # 用於確保在發佈訊息之前至少有一個訂閱者
            #   if self.xyz_vector_pub.get_num_connections()>0:
            #       self.xyz_vector_pub.publish(self.xyz_vector)
            #       self.sub_flag = True
            #   else:
            #       self.rate.sleep()
            ####test#####
          
            # print("-----------------------------------")
            # print("time is :",self.camera_info.header.seq)
            # print("pub pinhole0 :", pinhole0)
            # print("handover_vector_x: ", handover_vector_x)
            # print("handover_vector_y: ", handover_vector_y)
            # print("handover_vector_z: ", handover_vector_z)
        else:
          self.stop_update_target_bool_msg.data = True
      except:
            print("depth err")

      ### avoid x, y, z axis vectors being 0  還沒刪掉(1030)###
      # has_nan = False
      # for pose in self.xyz_vector.poses:
      #   if any(math.isnan(value) for value in [pose.position.x, pose.position.y, pose.position.z]):
      #     has_nan = True

      # if(not has_nan):
      #   self.xyz_vector_pub.publish(self.xyz_vector)
      #   self.xyz_vector.poses.clear()
      # else:
      #   print("one of xyz vector is nan")
      ####################################

    return annotated_image
  
  # 這邊還是相對camera link
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

  def image_cb(self, image_msg, info_msg, depth_msg):
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
    

    try:
      img = self.image_rgb.copy()      
      img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)   # 將 BGR 轉換成 RGB
      mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=img_rgb)
      results_landmark = self.detector_landmark.detect(mp_image)
      self.recognizer.recognize_async(mp_image, self.camera_info.header.seq)
      annotated_image = self.draw_and_pub_landmark(mp_image.numpy_view(), results_landmark)
      
      # pub for ukf(用來看是否要用此camera手的目標點)
      self.stop_update_target_pub.publish(self.stop_update_target_bool_msg)
      self.stop_update_target_bool_msg = True

      
      #fps
      # self.cTime =time.time()
      # fps = 1/(self.cTime-self.pTime)
      # self.pTime = self.cTime
      # cv2.putText(annotated_image, f"FPS:{int(fps)}", (30, 50), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1)


      if self.finish_one_gesture_recog_flag:
        cv2.putText(annotated_image, f"Gesture:{self.gesture_name}", (30, 70), cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 0), 1)
        self.finish_one_gesture_recog_flag = False
      else:
        self.gesture_name = "undetect"
      # cv2.imshow('MediaPipe Hands', cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB))
      # cv2.waitKey(1)
      # print(self.gesture_name)
      

      ros_annotated_image = self.bridge.cv2_to_imgmsg(annotated_image, "rgb8")
      self.mediapipe_img_pub.publish(ros_annotated_image)
    except CvBridgeError as e:
      print(e)
      print("publish img goes wrong!!!!!!!")
  
  # for suddenly shutdown
  def shutdown_node(self):
    rospy.loginfo("Shutting down ROS node")

    # 關閉 HandLandmarker 模型
    if hasattr(self, 'detector_landmark') and self.detector_landmark:
        self.detector_landmark.close()
        rospy.loginfo("HandLandmarker model closed")

    # 關閉 GestureRecognizer 模型
    if hasattr(self, 'recognizer') and self.recognizer:
        self.recognizer.close()
        rospy.loginfo("GestureRecognizer model closed")
    
    rospy.signal_shutdown("User requested shutdown")

def main():
  rospy.init_node('side_camera_demo_handpose', anonymous=True)
  hand_pose = handpose_mediapipe()

  # 註冊結束時的回調函數(好像沒啥用)
  rospy.on_shutdown(hand_pose.shutdown_node)
  rospy.spin()

if __name__ == '__main__':
  main()