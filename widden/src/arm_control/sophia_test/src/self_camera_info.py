#!/usr/bin/env python


# rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.03 image:=/camera/color/image_raw

# rosrun sophia_test self_camera_info.py
# ROS_NAMESPACE=my_camera rosrun image_proc image_proc

# rosrun image_view image_view image:=my_camera/image_rect_color

import rospy
from sensor_msgs.msg import CameraInfo,Image

pub_info = rospy.Publisher('/my_camera/camera_info', CameraInfo, queue_size=10)
pub_img = rospy.Publisher('/my_camera/image_raw', Image, queue_size=10)
camera_info = CameraInfo()

def callback(data):
    # rospy.loginfo("heard image")
    camera_info.header=data.header
    pub_info.publish(camera_info)

    pub_img.publish(data)

def set_info_1080p():
    camera_info.header.frame_id = 'camera_color_optical_frame'
    camera_info.width = int(1920)
    camera_info.height = int(1080)
    camera_info.distortion_model = 'plumb_bob'
    
    camera_info.K = [1391.421392437757, 0.0, 984.8817867585748, 0.0, 1399.062332428129, 546.0107499189386, 0.0, 0.0, 1.0]
    camera_info.D = [0.10677420376999978, -0.22224956982603475, 0.00234550219175402, 0.0046939168627538, 0.0]
    camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    camera_info.P = [1390.0489501953125, 0.0, 998.0423795295574, 0.0, 0.0, 1421.0853271484375, 547.7746690318418, 0.0, 0.0, 0.0, 1.0, 0.0]

# def set_info_720p():
#     camera_info.header.frame_id = 'camera_color_optical_frame'
#     camera_info.width = int(1280)
#     camera_info.height = int(720)
#     camera_info.distortion_model = 'plumb_bob'
    
#     camera_info.K = [903.1720301666824, 0.0, 654.5506257192524, 0.0, 904.8970334471217, 345.7207745509376, 0.0, 0.0, 1.0]
#     camera_info.D = [0.12114029471124542, -0.23592972745708865, -0.0026820344271046514, 0.0015648324575131707, 0.0]
#     camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
#     camera_info.P = [903.8492431640625, 0.0, 658.8623817690459, 0.0, 0.0, 919.6047973632812, 344.3231851657765, 0.0, 0.0, 0.0, 1.0, 0.0]
def set_info_720p(): #02/14
    camera_info.header.frame_id = 'camera_color_optical_frame'
    camera_info.width = int(1280)
    camera_info.height = int(720)
    camera_info.distortion_model = 'plumb_bob'
    
    # camera_info.K = [915.1245674387874, 0.0, 648.4402475617848, 0.0, 920.4451913182177, 357.4599673997687, 0.0, 0.0, 1.0]
    # camera_info.D = [0.09942966631127782, -0.19216862889505149, 0.0023155574977127397, 0.0028997033758752853, 0.0]
    # camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
    # camera_info.P = [917.4024658203125, 0.0, 653.4584514520029, 0.0, 0.0, 934.0859375, 358.6471348707564, 0.0, 0.0, 0.0, 1.0, 0.0]
    # camera_info.K = [914.9116702340889, 0.0, 630.6161787648197, 0.0, 916.6565585020683, 364.15261261000234, 0.0, 0.0, 1.0]
    # camera_info.D = [0.10821762584724219, -0.2299419680595839, 0.005227491130920149, -0.003987350113684727, 0.0]
    # camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    # camera_info.P = [912.0048217773438, 0.0, 623.2067775274663, 0.0, 0.0, 930.3936157226562, 366.8604820040691, 0.0, 0.0, 0.0, 1.0, 0.0]

    camera_info.K = [913.5582647764578, 0.0, 649.481141569651, 0.0, 916.2389898284449, 349.42891148576695, 0.0, 0.0, 1.0]
    camera_info.D = [0.09316626302912373, -0.18761267746481197, 0.001268223636410371, -0.00041414217769421405, 0.0]
    camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camera_info.P = [914.064453125, 0.0, 649.7969863369362, 0.0, 0.0, 927.0164794921875, 350.069726126756, 0.0, 0.0, 0.0, 1.0, 0.0]

def listener():
    set_info_720p()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, callback)
    rospy.spin()

# def talker():
#     pub = rospy.Publisher('/camera/color/camera_info/calib', CameraInfo, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(30) # 10hz

#     camera_info = CameraInfo()
#     # store info without header
#     camera_info.header.frame_id = 'camera_color_optical_frame'
#     camera_info.width = int(1920)
#     camera_info.height = int(1080)
#     camera_info.distortion_model = 'plumb_bob'
    
#     camera_info.K = [1384.9298208541372, 0.0, 985.4716646116344, 0.0, 1384.6685357604483, 549.6928889552491, 0.0, 0.0, 1.0]
#     camera_info.D = [0.11084953847688737, -0.19306834260709727, 0.0033638012729857564, 0.004100204354695286, 0.0]
#     camera_info.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
#     camera_info.P = [1396.4166259765625, 0.0, 996.0548925384865, 0.0, 0.0, 1410.624755859375, 552.5184261636095, 0.0, 0.0, 0.0, 1.0, 0.0]

#     while not rospy.is_shutdown():
        
#         pub.publish(camera_info)
#         rate.sleep()

if __name__ == '__main__':
    try:
        listener()
        # talker()
    except rospy.ROSInterruptException:
        pass