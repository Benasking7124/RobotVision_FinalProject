#!/usr/bin/env python3

import rospy
from final_project.msg import Int2DArray, IntArray
from std_msgs.msg import Float32
import numpy as np
from scipy.linalg import logm
import random

processed = [0, 0]   #first is convex, second is concave

point_cloud_convex = np.array([[]])
point_cloud_concave = np.array([[]])
Transformation = np.array([[]])

rotation_pub = rospy.Publisher('rotation_angle', Float32, queue_size=10)
rotation_angle = Float32()

def get_point_cloud_convex(array):
    global processed
    global rotation_angle
    if processed[0] == 0:  #Only process once
        processed[0] = 1

        #Decrypt point cloud
        point_cloud_list = []
        for i in range(len(array.data)):
            point_cloud_list.append(array.data[i].data)
        global point_cloud_convex
        global point_cloud_concave
        point_cloud_convex = np.transpose(np.asfarray(point_cloud_list))

        #Downsizing
        if processed[1] == 0:   #Early return if concave haven't been processed
            return
        n_difference = point_cloud_convex.shape[1] - point_cloud_concave.shape[1]
        if n_difference > 0:
            point_cloud_convex = randomly_downsizing(point_cloud_convex, n_difference)
        elif n_difference < 0:
            point_cloud_concave = randomly_downsizing(point_cloud_concave, -n_difference)
        
        #Do ICP
        global Transformation
        Transformation = icp_simple(point_cloud_convex, point_cloud_concave, 100)
        processed = [2, 2]
        rotation_angle.data = logm(Transformation[0:2, 0:2])[1, 0]
        
    if processed == [2, 2]:
        rotation_pub.publish(rotation_angle)
        rospy.loginfo("Rotation Angle:" + str(rotation_angle.data))


def get_point_cloud_concave(array):
    global processed
    global rotation_angle
    if processed[1] == 0:   #Only process once
        processed[1] = 1

        #Decrypt point cloud
        point_cloud_list = []
        for i in range(len(array.data)):
            point_cloud_list.append(array.data[i].data)
        global point_cloud_convex
        global point_cloud_concave
        point_cloud_concave = np.transpose(np.asfarray(point_cloud_list))
        
        #Downsizing
        if processed[0] == 0:   #Early return if convex haven't been processed
            return
        n_difference = point_cloud_convex.shape[1] - point_cloud_concave.shape[1]
        if n_difference > 0:
            point_cloud_convex = randomly_downsizing(point_cloud_convex, n_difference)
        elif n_difference < 0:
            point_cloud_concave = randomly_downsizing(point_cloud_concave, -n_difference)

        #Do ICP
        global Transformation
        Transformation = icp_simple(point_cloud_convex, point_cloud_concave, 100)
        processed = [2, 2]
        rotation_angle.data = logm(Transformation[0:2, 0:2])[1, 0]
    
    if processed == [2, 2]:
        rotation_pub.publish(rotation_angle)
        rospy.loginfo("Rotation Angle:" + str(rotation_angle.data))

def randomly_downsizing(point_cloud, N):
    removed = random.sample(range(0, point_cloud.shape[1]), N)
    return np.delete(point_cloud, removed, axis=1)

def pq2tr(P, Q):

    P_mean = np.mean(P, axis=1).reshape(-1, 1)
    Q_mean = np.mean(Q, axis=1).reshape(-1, 1)

    W = np.zeros([2, 2])
    for i in range(P.shape[1]):
        W += (P[:, i:(i+1)] - P_mean) *  (Q[:, i:(i+1)] - Q_mean).reshape(1, -1)
    U, _, Vh = np.linalg.svd(W)
    V = Vh.transpose()

    R = np.matmul(V, U.transpose())
    t = Q_mean - P_mean
    T = np.block([[R, t], [np.zeros(2), 1]])
    return T

def closest(A, B):
    N = A.shape[1]
    K = np.zeros(N)
    D1 = np.zeros(N)
    for i in range(N):
        #Calculate distance (D)
        D = B - A[:, i:(i+1)] * np.ones(N)
        #Find minimum norm and index
        Vec_norm = np.zeros(N)
        for j in range(N):
            Vec_norm[j] = np.linalg.norm(D[:, j:(j+1)])
        D1[i] = min(Vec_norm)
        K[i] = np.argmin(Vec_norm)
    return K, D1

def icp_simple(P, Q, N):
    Q_temp = Q
    for i in range(N):
        corresp , _= closest(P, Q_temp)
        corresp = np.array(corresp).astype(int)
        Transform = pq2tr(P, Q_temp[:, corresp])
        Q_temp_H = np.block([[Q_temp], [np.ones(Q_temp.shape[1])]])
        Q_temp_H = np.matmul(np.linalg.inv(Transform), Q_temp_H)
        Q_temp = Q_temp_H[0:2, :]
        print(i)

    Transform = pq2tr(P, Q[:, corresp])
    return Transform

# Main
rospy.init_node('icp', anonymous=False)

convex_sub = rospy.Subscriber('point_cloud_convex', Int2DArray, get_point_cloud_convex)
concave_sub = rospy.Subscriber('point_cloud_concave', Int2DArray, get_point_cloud_concave)

# P = [(0.5377, 1.8339, -2.2588), (0.8622, 0.3188, -1.3077), (2.7694, -1.3499, 3.0349), (-0.4336, 0.3426, 3.5784), (0.7254, -0.0631, 0.7147)]
# P_array = np.transpose(np.asfarray(P))
# Q = [(0.5340, 0.4029, 0.0847), (-0.3943, 1.7932, -0.6301), (0.9526, -0.0766, 2.0467), (0.3811, -0.1301, 5.1092), (3.7270, -0.8011, 3.7773)]
# Q_array = np.transpose(np.asfarray(Q))

# print(icp_simple(P_array, Q_array, 10))

rospy.spin()