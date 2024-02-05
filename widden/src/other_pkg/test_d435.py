# import some common libraries
import numpy as np
import cv2
import matplotlib.pyplot as plt
import os


#  import some ros libraries and others
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge, CvBridgeError
import message_filters

tack_pic = True
bridge = CvBridge()
image_rgb = None
image_dep = None

def image_cb(image_msg, depth_msg):
    global tack_pic
    global image_rgb
    global image_dep
    try:
        image_rgb = bridge.imgmsg_to_cv2(image_msg, "bgr8") # image_rgb: 480*640
        image_dep = bridge.imgmsg_to_cv2(depth_msg, "passthrough").copy() # /10 = cm
        cv2.imwrite("image_rgb.jpg", image_rgb)

            
    except CvBridgeError as e:
        print(e)


if __name__ == "__main__":
    
    rospy.init_node('rotten_detection', anonymous=True)
        
    image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    time_sync = message_filters.TimeSynchronizer([image_sub, depth_sub], 10)
    time_sync.registerCallback(image_cb)
    
    rospy.spin()