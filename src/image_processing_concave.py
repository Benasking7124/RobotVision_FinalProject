#!/usr/bin/env python3

import rospy

from std_msgs.msg import Int16
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
from final_project.msg import Int2DArray, IntArray
import math
import cv2

take_pic = 1 #0: Do not take pic; 1: Take pic; 2:Taken pic
point_cloud_concave = []
point_cloud_concave_msg = Int2DArray()
bridge = CvBridge()
pub_pc_concave = rospy.Publisher('point_cloud_concave', Int2DArray, queue_size=10)
pub_img = rospy.Publisher('processed_image_concave', Image, queue_size=10)
def take_picture(data):
    take_pic = data.data

def image_process_concave(image):
    global take_pic
    if take_pic == 0:
        return
    elif take_pic == 2:
        pub_pc_concave.publish(point_cloud_concave_msg)
        return
    take_pic = 2

    #convert to opencv format
    CV2_img = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
    
    CV2_img = cv2.flip(CV2_img, 0)

    # Use color to cut out the sub image to get shape feature
    lower_blue = np.array([0, 0, 70])
    upper_blue = np.array([255, 255, 255])
    img_mask = cv2.inRange(CV2_img, lower_blue, upper_blue)
    contours, hierarchy = cv2.findContours(img_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    sub_x, sub_y, sub_w, sub_h = 0, 0, 0, 0
    for cnt in contours:
        epsilon = 0.01 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt,epsilon,True)
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(cnt)
            if (w < 200 and w > 30) and (h < 200 and h > 30):
                sub_x, sub_y, sub_w, sub_h = x, y, w, h


    # Cut out the sub image, extract the feature
    sub_img = CV2_img[sub_y:(sub_y + sub_h), sub_x:(sub_x + sub_w)]
    sub_gray = cv2.cvtColor(sub_img, cv2.COLOR_RGB2GRAY)
    ret,thresh = cv2.threshold(sub_gray,100,255,0)

    # Convert to point cloud
    img_msg = bridge.cv2_to_imgmsg(thresh, encoding="passthrough")
    point_cloud_concave.clear()
    for i in range(len(img_msg.data)):
        if img_msg.data[i] == 0:
            point_cloud_concave.append([i % img_msg.width, int(i /img_msg.width)])
    
    # Filter out
    x_mean, y_mean = np.mean(point_cloud_concave, axis=0)
    remove_list = []
    for p in point_cloud_concave:
        if math.sqrt(((p[0] - x_mean) ** 2 + (p[1] - y_mean) ** 2)) > 20:
            remove_list.append(p)
    for r in remove_list:
        point_cloud_concave.remove(r)

    # Convert to ROS message type
    point_cloud_concave_msg.data.clear()
    for i in range(len(point_cloud_concave)):
        array = IntArray()
        array.data = point_cloud_concave[i]
        point_cloud_concave_msg.data.append(array)
    pub_pc_concave.publish(point_cloud_concave_msg)

    # Print the point cloud(for debug)
    for x in point_cloud_concave:
        sub_img = cv2.circle(sub_img, (x[0],x[1]), radius=0, color=(0, 0, 255), thickness=-1)
    img_msg = bridge.cv2_to_imgmsg(sub_img, encoding="passthrough")
    pub_img.publish(img_msg)

# Main
rospy.init_node('image_processing_concave', anonymous=False)

command_concave_sub = rospy.Subscriber('take_picture_concave', Int16, take_picture)
image1_concave_sub = rospy.Subscriber('image_concave', Image, image_process_concave)

rospy.spin()
