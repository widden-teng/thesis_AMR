#!/usr/bin/env python3
import rospy
import cv_bridge
import ros_numpy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
import numpy as np
import cv2
import torch
from helper_CSAILVision.lib.segmentation import hand_segmentation, module_init
import message_filters
from std_msgs.msg import Bool



    



class Egohands:
    def __init__(self):        

        # Init
        self.bridge = cv_bridge.CvBridge()
        self.segmentation_module = module_init()
        torch.cuda.set_device(0)
        
        # # Pub
        self.pub_visualization = rospy.Publisher("/egohands/hand_origin_img", Image, queue_size=1)
        self.pub_mask_fastsam = rospy.Publisher("/egohands/mask_with_fastsam", Image, queue_size=1)
        self.pub_release_gripper = rospy.Publisher("release_gripper_semantic_segmentation",Bool, queue_size=1)


        # # Sub     

        # 用來同步
        # 這邊的sub並沒有宣告callback function
        self.image_sub = message_filters.Subscriber('/side_camera/color/image_raw/compressed', CompressedImage)
        self.camera_info_sub = message_filters.Subscriber('/side_camera/color/camera_info', CameraInfo)
        self.depth_sub = message_filters.Subscriber('/side_camera/aligned_depth_to_color/image_raw', Image)
        #用來判斷三個msg是否都有傳到topic(同步)  
        time_sync = message_filters.TimeSynchronizer([self.image_sub, self.camera_info_sub, self.depth_sub], 1)
        time_sync.registerCallback(self._callback) 

        # get fastsam mask
        rospy.Subscriber("FastSAM_final_mask_result_img", Image, self.startCB, queue_size=1)


        # class variable
        self.last_handobject_aera = 0
        self.handobject_aera_delta_rate = 1
        self.grab_obj_frame = 0
        self.fastsam_mask = None
        self.get_fastsam_flag = False

        # Feedback
        print("Hand segmentation publisher up and running")

    # 用來判斷fastsam是否偵測完並發出mask
    def startCB(self, mask_img):
        target_color = np.array([255, 255, 255])
        mask_img_temp = self.bridge.imgmsg_to_cv2(mask_img, "bgr8") 
        self.fastsam_mask = np.any(mask_img_temp == target_color, axis=-1)
        # (之後可以關掉get_fastsam_flag的部份, fastsam_mask的default為None, 直接拿去egohand 出來不會有人手面積!!)
        self.get_fastsam_flag = True
        


    
    # Callback function
    def _callback(self, msg, info_msg, depth_msg):
        # 之後可以關掉get_fastsam_flag的部份, fastsam_mask的default為None, 直接拿去egohand 出來不會有人手面積!!
        if self.get_fastsam_flag:
            t_start = rospy.get_time()

            # Get image
            image = cv2.cvtColor(self.bridge.compressed_imgmsg_to_cv2(msg), cv2.COLOR_BGR2RGB)
            egohand_origin_img = image.copy()
            
            # Calculate Mask                 
            mask = hand_segmentation(image, self.segmentation_module)

            # add mask
            egohand_origin_img[:,:,0][mask == 0] = 0
            egohand_origin_img[:,:,1][mask == 0] = 0
            egohand_origin_img[:,:,2][mask == 0] = 0
            egohand_origin_img = cv2.cvtColor(egohand_origin_img, cv2.COLOR_BGR2RGB)

            img_mask_fastsam = egohand_origin_img.copy()
            img_mask_fastsam[self.fastsam_mask] = [0, 0, 0]

            # 计算非黑色pixel的aera
            current_handobject_aera = np.sum(np.any(img_mask_fastsam != [0, 0, 0], axis=-1))
            
            if current_handobject_aera == 0:
                self.handobject_aera_delta_rate = 1
                self.grab_obj_frame = 0
            else:
                self.handobject_aera_delta_rate = abs(current_handobject_aera - self.last_handobject_aera) / current_handobject_aera

            # 先設aera變化律小於1% 
            if self.handobject_aera_delta_rate < 0.03 :
                self.grab_obj_frame = self.grab_obj_frame +1
            else:
                self.grab_obj_frame = 0



            # 打印结果
            print("-------------------------------------------------------------")
            print("current_handobject_aera:", current_handobject_aera)
            print("handobject_aera_delta_rate:", self.handobject_aera_delta_rate)
            print("self.grab_obj_frame: ", self.grab_obj_frame)

            if(self.grab_obj_frame >= 3):
                print("release gripper !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                self.pub_release_gripper.publish(Bool(data=True))
            print("-------------------------------------------------------------")

            self.last_handobject_aera = current_handobject_aera


            # 將影像傳到topic上
            try:
                self.pub_mask_fastsam.publish(ros_numpy.msgify(Image, img_mask_fastsam, encoding='8UC3'))
                self.pub_visualization.publish(ros_numpy.msgify(Image, egohand_origin_img, encoding='8UC3'))
            except cv_bridge.CvBridgeError as e:
                print(e)
            
            # Publish results
            print('Hand detection successful. Current Hz-rate:\t' + str(1/(rospy.get_time() - t_start)))
        else:
            print("wait fastsam mask")



if __name__ == '__main__':

    rospy.init_node('egohands_publisher')
    body = Egohands()
    rospy.spin()