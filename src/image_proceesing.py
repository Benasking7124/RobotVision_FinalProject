#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge

import cv2

take_pic = True
bridge = CvBridge()
pub1 = rospy.Publisher('processed_image_concave', Image, queue_size=10)
def take_picture(data):
    take_pic = data

def image_process(image):
    img_msg = image

    if take_pic == False:
        return
    
    #convert to opencv format
    CV2_img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
    
    CV2_img = cv2.flip(CV2_img, 0)
    CV2_gray = cv2.cvtColor(CV2_img, cv2.COLOR_RGB2GRAY)
    # ret,thresh = cv2.threshold(CV2_gray,205,255,0)
    # thresh = cv2.adaptiveThreshold(CV2_gray, 250, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
    contours, hierarchy = cv2.findContours(CV2_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    x, y, w, h = 0, 0, 0, 0
    for cnt in contours:
        epsilon = 0.01 * cv2.arcLength(cnt, True)
        # rospy.loginfo(epsilon)
        approx = cv2.approxPolyDP(cnt,epsilon,True)
        # rospy.loginfo("approx: " + str(len(approx)))
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(cnt)
            # cv2.rectangle(CV2_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # rospy.loginfo(str(x) + " " + str(y) + " " + str(w) + " " + str(h))
    
    sub_img = CV2_gray[y:(y+h), x:(x+w)]
    # contours, hierarchy = cv2.findContours(sub_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    # for cnt in contours:
    #     epsilon = 0.01 * cv2.arcLength(cnt, True)
    #     # rospy.loginfo(epsilon)
    #     approx = cv2.approxPolyDP(cnt,epsilon,True)
    #     # rospy.loginfo("approx: " + str(len(approx)))
    #     if len(approx) == 4:
    #         x, y, w, h = cv2.boundingRect(cnt)
    #         cv2.rectangle(sub_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
    #         rospy.loginfo(str(x) + " " + str(y) + " " + str(w) + " " + str(h))

    kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]], np.float32)
    img_sh = cv2.filter2D(sub_img, -1, kernel=kernel)




    edges = cv2.Canny(img_sh,120,150)
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    rospy.loginfo(len(contours))
    for cnt in contours:
        epsilon = 0.01 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt,epsilon,True)
        # rospy.loginfo("approx: " + str(len(approx)))
        # if len(approx) == 4:
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(sub_img, (x, y), (x + w, y + h), (0, 255, 0), 5)
        rospy.loginfo(str(x) + " " + str(y) + " " + str(w) + " " + str(h))
    #output to ROS format to publish
    img_msg = bridge.cv2_to_imgmsg(sub_img, encoding="passthrough")
    rospy.loginfo('pub')
    pub1.publish(img_msg)

rospy.init_node('image_proccesing_concave', anonymous=False)


command_concave_sub = rospy.Subscriber('take_picture_concave', Bool, take_picture)
image1_concave_sub = rospy.Subscriber('image_concave', Image, image_process)

rospy.spin()
