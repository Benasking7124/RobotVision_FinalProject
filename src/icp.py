#!/usr/bin/env python3

import rospy
from final_project.msg import Int2DArray, IntArray
import numpy as np

start_calculate = 0
point_cloud_convex = []
point_cloud_concave = []
def get_point_cloud_convex(array):
    # rospy.loginfo("convex" + str(len(array.data)))
    point_cloud_convex.clear()
    for i in range(len(array.data)):
        point_cloud_convex.append(array.data[i].data)
    rospy.loginfo(len(point_cloud_convex))
    rospy.loginfo(point_cloud_convex)

def get_point_cloud_concave(array):
    # rospy.loginfo("concave" + str(len(array.data)))
    point_cloud_concave.clear()
    for i in range(len(array.data)):
        point_cloud_concave.append(array.data[i].data)
    rospy.loginfo(len(point_cloud_concave))
    rospy.loginfo(point_cloud_concave)

# Main
rospy.init_node('icp', anonymous=False)

convex_sub = rospy.Subscriber('point_cloud_convex', Int2DArray, get_point_cloud_convex)
concave_sub = rospy.Subscriber('point_cloud_concave', Int2DArray, get_point_cloud_concave)

rospy.spin()