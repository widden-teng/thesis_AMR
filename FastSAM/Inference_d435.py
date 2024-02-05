import argparse
from fastsam import FastSAM, FastSAMPrompt 
import ast
import torch
from PIL import Image as PILImage
from utils.tools import convert_box_xywh_to_xyxy
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from geometry_msgs.msg import Point, PoseArray, Pose
from std_msgs.msg import Bool
import numpy as np
import time

class FastSAM_with_realsense(object):
  def __init__(self, args):

    ### load model
    self.args = args
    self.model = FastSAM(self.args.model_path)
    self.args.point_prompt = ast.literal_eval(self.args.point_prompt)
    self.args.box_prompt = convert_box_xywh_to_xyxy(ast.literal_eval(self.args.box_prompt))
    self.args.point_label = ast.literal_eval(self.args.point_label)

    ### For sync RGBD information
    self.bridge = CvBridge()
    self.image_rgb = None
    self.image_depth = None
    self.camera_info = None
    self.image_sub = message_filters.Subscriber('/side_camera/color/image_raw', Image)
    self.camera_info_sub = message_filters.Subscriber('/side_camera/color/camera_info', CameraInfo)
    self.depth_sub = message_filters.Subscriber('/side_camera/aligned_depth_to_color/image_raw', Image)
    #用來判斷三個msg是否都有傳到topic(同步)  
    time_sync = message_filters.TimeSynchronizer([self.image_sub, self.camera_info_sub, self.depth_sub], 10)
    time_sync.registerCallback(self.image_cb)   

    ### Pub
    self.xyz_vector_pub = rospy.Publisher("handpose_cameralink",PoseArray, queue_size=1)
    self.limit_depth_img_pub = rospy.Publisher("limit_depth_img", Image, queue_size = 1)
    self.result_pub = rospy.Publisher("FastSAM_result_img", Image, queue_size = 1)
    self.final_mask_result_pub = rospy.Publisher("FastSAM_final_mask_result_img", Image, queue_size = 1)
    self.stop_update_target_pub = rospy.Publisher("stop_update_target_flag",Bool, queue_size=1)

    ### class variable
    self.result_img_list = []
    self.number_of_result = 0

  def image_cb( self, image_msg, info_msg, depth_msg):
    try:
        #cv2通常吃bgr8
        self.image_rgb = self.bridge.imgmsg_to_cv2(image_msg, "bgr8") 
        self.image_masked = self.image_rgb.copy() 
        self.camera_info = info_msg
        self.image_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough").copy()

        ### 兩種檔depth擋極端值的方法 ###
        ### 法一
        # 當沒吃到深度時, 深度為0(可能過熱或那個點沒吃到ex:太遠太小)
        #self.image_depth[np.isnan(self.image_depth)] = 0.

        ### 法二
        #沒有pixel會是0(有誤的深度都會變成65535)
        self.image_depth[np.isnan(self.image_depth)] = 0.
        zero_mask = (self.image_depth == 0 )
        #np.iinfo(self.image_depth.dtype).max為 65535
        self.image_depth[zero_mask] = np.iinfo(self.image_depth.dtype).max

        ####################################################

        ### 利用深度來獲得扣掉nan(0 or 65535) 且小於特定距離的mask ###
        # 後面fastsam再做最終mask濾除時會使用, 給450～500 就差不多了
        depth_useless_mask = (self.image_depth > 500) | (self.image_depth == 0)
        self.image_masked[depth_useless_mask] = 0
        ####################################################

    except CvBridgeError as e:
        print(e)

    # 将OpenCV图像转换为PIL图像
    self.input = PILImage.fromarray(cv2.cvtColor(self.image_rgb, cv2.COLOR_BGR2RGB))
    self.input = self.input.convert("RGB")
    
    # 進FastSAM model
    everything_results = self.model(
        self.input,
        device=self.args.device,
        retina_masks=self.args.retina,
        imgsz=self.args.imgsz,
        conf=self.args.conf,
        iou=self.args.iou    
        )
    bboxes = None
    points = None
    point_label = None
    prompt_process = FastSAMPrompt(self.input, everything_results, device=self.args.device)
    if self.args.box_prompt[0][2] != 0 and self.args.box_prompt[0][3] != 0:
            ann = prompt_process.box_prompt(bboxes=self.args.box_prompt)
            bboxes = self.args.box_prompt
    elif self.args.text_prompt != None:
        ann = prompt_process.text_prompt(text=self.args.text_prompt)
    elif self.args.point_prompt[0] != [0, 0]:
        ann = prompt_process.point_prompt(
            points=self.args.point_prompt, pointlabel=self.args.point_label
        )
        points = self.args.point_prompt
        point_label = self.args.point_label
    else:
        ann = prompt_process.everything_prompt()

    # input額外加入depth 以及 depth_useless_mask
    result_img = prompt_process.plot(
        annotations=ann,
        output_path=self.args.output+self.args.img_path.split("/")[-1],
        depth = self.image_depth,
        depth_useless_mask = depth_useless_mask,
        bboxes = bboxes,
        points = points,
        point_label = point_label,
        withContours=self.args.withContours,
        better_quality=self.args.better_quality,
        mask_random_color = self.args.randomcolor
    )

    # 累積前n幀的mask
    self.result_img_list.append(result_img)
    self.number_of_result = self.number_of_result + 1

    ################# 最終mask的篩選(三種方法) ##############################
    # 先建立全白的mask. 要的部份的pixel變成黑色
    
    ### 法一 : 只要有出現就算 ###
    # if (self.number_of_result == 5):
    #     final_result =  np.ones_like(self.result_img_list[0])*255
    #     for temp_image in self.result_img_list:
    #         final_result[(temp_image > 0).all(axis=-1)] = [0, 0, 0]
    #     try:
    #         print("sending final mask img")
    #         self.final_mask_result_pub.publish(self.bridge.cv2_to_imgmsg(final_result, "bgr8"))
    #     except CvBridgeError as e:
    #         print(e)
    ######

    ### 法二 : 每幀都要出現才算 ###
    # if (self.number_of_result == 5):
    #     # 初始化一个标志数组，用于跟踪所有照片中都存在的像素位置
    #     common_pixels = np.ones((480, 640), dtype=bool)

    #     final_result =  np.ones_like(self.result_img_list[0])*255
    #     for temp_image in self.result_img_list:
    #         common_pixels = np.logical_and(common_pixels, temp_image[:, :, 0] > 0)
    #     final_result[common_pixels] = [0, 0, 0]
    #     try:
    #         print("sending final mask img")
    #         self.final_mask_result_pub.publish(self.bridge.cv2_to_imgmsg(final_result, "bgr8"))
    #     except CvBridgeError as e:
    #         print(e)
    ######

    ### 法三 : 該pixel有顏色的比例要大於n才算 ###
    if (self.number_of_result == 5):
        # 初始化一个标志数组，用于跟踪所有照片中都存在的像素位置
        pixel_counts = np.ones((480, 640), dtype=int)

        final_result =  np.ones_like(self.result_img_list[0])*255
        for temp_image in self.result_img_list:
            # 更新出现次数
            pixel_counts += (temp_image[:, :, 0] > 0).astype(int)
        
        
        # 計算每个pixel的出现概率
        pixel_probabilities = pixel_counts / len(self.result_img_list)

        # 將概率大于60%的像素位置设置为黑色(可根據夾取啥物品決定)
        final_result[pixel_probabilities > 0.6] = [0, 0, 0]
        try:
            print("sending final mask img")
            self.final_mask_result_pub.publish(self.bridge.cv2_to_imgmsg(final_result, "bgr8"))
        except CvBridgeError as e:
            print(e)

        ### 測試用(每n次會更新最終mask, 並pub出去)
        # self.number_of_result = 0
        # self.result_img_list.clear()
            
    ########################################################################

    ### 顯示影像(這邊顯示mask)
    # cv2.imshow('result', result_img)
    # cv2.waitKey(3)

    # 將影像傳到topic上
    try:
        self.limit_depth_img_pub.publish(self.bridge.cv2_to_imgmsg(self.image_masked, "bgr8"))
        self.result_pub.publish(self.bridge.cv2_to_imgmsg(result_img, "bgr8"))
    except CvBridgeError as e:
        print(e)

################################################################################################################################################
        
# load 參數 
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model_path", type=str, default="./weights/new_weight/FastSAM-x.pt", help="model"
    )
    parser.add_argument(
        "--img_path", type=str, default="./images/output_img.jpg", help="path to image file"
    )
    parser.add_argument("--imgsz", type=int, default=640, help="image size")
    parser.add_argument(
        "--iou",
        type=float,
        default=0.9,
        help="iou threshold for filtering the annotations",
    )
    parser.add_argument(
        "--text_prompt", type=str, default=None, help='use text prompt eg: "a dog"'
    )
    parser.add_argument(
        "--conf", type=float, default=0.4, help="object confidence threshold"
    )
    parser.add_argument(
        "--output", type=str, default="./output/", help="image save path"
    )
    # 這邊有點奇怪, 不管輸入是啥, 都會變成True(沒輸入的話就不會錯-False) -> 因boolean不會解析True或 False, 只要有輸入就是True
    # 只要是False, 不管是否為 segmentation everything, 皆只有一個顏色
    parser.add_argument(
        "--randomcolor", type=bool, default=True, help="mask random color"
    )
    parser.add_argument(
        "--point_prompt", type=str, default="[[0,0]]", help="[[x1,y1],[x2,y2]]"
    )
    parser.add_argument(
        "--point_label",
        type=str,
        default="[0]",
        help="[1,0] 0:background, 1:foreground",
    )
    parser.add_argument("--box_prompt", type=str, default="[[0,0,0,0]]", help="[[x,y,w,h],[x2,y2,w2,h2]] support multiple boxes")
    parser.add_argument(
        "--better_quality",
        type=str,
        default=False,
        help="better quality using morphologyEx",
    )
    device = torch.device(
        "cuda"
        if torch.cuda.is_available()
        else "mps"
        if torch.backends.mps.is_available()
        else "cpu"
    )
    parser.add_argument(
        "--device", type=str, default=device, help="cuda:[0,1,2,3,4] or cpu"
    )
    # 當沒有去背景時會影響mask的結果(不知為啥，感覺應該是不會)
    parser.add_argument(
        "--retina",
        type=bool,
        default=True,
        help="draw high-resolution segmentation masks",
    )
    parser.add_argument(
        "--withContours", type=bool, default=False, help="draw the edges of the masks"
    )
    return parser.parse_args()

def main():
    rospy.init_node('Inference_d435', anonymous=True)
    args = parse_args()
    fastsam_realsense = FastSAM_with_realsense(args)
    rospy.spin()

if __name__ == "__main__":
    main()

