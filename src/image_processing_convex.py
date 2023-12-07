#!/usr/bin/env python3

import rospy

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
from final_project.msg import Int2DArray, IntArray

import cv2

take_pic = True
point_cloud_convex = []
point_cloud_convex_msg = Int2DArray()
bridge = CvBridge()
pub_pc_convex = rospy.Publisher('point_cloud_convex', Int2DArray, queue_size=10)
pub_img = rospy.Publisher('processed_image_convex', Image, queue_size=10)
def take_picture(data):
    take_pic = data

def image_process_convex(image):

    if take_pic == False:
        return
    
    #convert to opencv format
    CV2_img = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    
    CV2_img = cv2.flip(CV2_img, 0)

    # Use color to cut out the sub image to get shape feature
    lower_orange = np.array([50, 50, 0])
    upper_orange = np.array([255, 255, 255])
    img_mask = cv2.inRange(CV2_img, lower_orange, upper_orange)
    contours, hierarchy = cv2.findContours(img_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    sub_x, sub_y, sub_w, sub_h = 0, 0, 0, 0
    for cnt in contours:
        epsilon = 0.01 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt,epsilon,True)
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(cnt)
            if (w < 200 and w > 30) and (h < 200 and h > 30):
                sub_x, sub_y, sub_w, sub_h = x, y, w, h
                # rospy.loginfo(str(x) + " " + str(y))


    # # Cut out the sub image, extract the feature
    sub_img = CV2_img[sub_y:(sub_y + sub_h), sub_x:(sub_x + sub_w)]
    sub_gray = cv2.cvtColor(sub_img, cv2.COLOR_RGB2GRAY)
    ret,thresh = cv2.threshold(sub_gray, 90,255,0)

    # # Convert to point cloud
    img_msg = bridge.cv2_to_imgmsg(thresh, encoding="passthrough")
    # rospy.loginfo(len(img_msg.data))
    point_cloud_convex.clear()
    for i in range(len(img_msg.data)):
        # rospy.loginfo(img_msg.data[i])
        if img_msg.data[i] == 0:
            point_cloud_convex.append([i % img_msg.width, int(i /img_msg.width)])
    point_cloud_convex_msg.data.clear()
    for i in range(len(point_cloud_convex)):
        array = IntArray()
        array.data = point_cloud_convex[i]
        point_cloud_convex_msg.data.append(array)
    pub_pc_convex.publish(point_cloud_convex_msg)
    
    # Print the point cloud(for debug)
    for x in point_cloud_convex:
        sub_img = cv2.circle(sub_img, (x[0],x[1]), radius=0, color=(0, 0, 255), thickness=-1)
    img_msg = bridge.cv2_to_imgmsg(sub_img, encoding="passthrough")
    pub_img.publish(img_msg)

# Main
rospy.init_node('image_proccesing_convex', anonymous=False)

command_convex_sub = rospy.Subscriber('take_picture_convex', Bool, take_picture)
image1_convex_sub = rospy.Subscriber('image_convex', Image, image_process_convex)

rospy.spin()
