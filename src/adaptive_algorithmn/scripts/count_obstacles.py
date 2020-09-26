#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy,os,math
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from std_msgs.msg import String
import time

from sklearn.cluster import DBSCAN,KMeans
import numpy as np

'''
author:zlb
实现的功能:
    节点名称:count_obstacles
    订阅 /self_costmap 
    发布 /number_obstacles, String, 障碍物个数 障碍物中心 障碍物半径 障碍物实际占据像素点个数,
    "7 - [[82, 21, 10, 317], [131, 124, 10, 317], [44, 142, 10, 317], [38, 52, 10, 317], [165, 145, 10, 317], [30, 106, 10, 317], [94, 115, 17, 740]]"
'''


data = []

msg_number_obstacles = "" # 这个就是 self_costmap
costmap_height = 200 # 全局代价地图的像素长
costmap_width = 200 # 全局代价地图的像素宽

start_calculate_flag = False
def start_calculate(msg):
    global start_calculate_flag
    rospy.loginfo("in start_calculate")
    start_calculate_flag = True

def pub_count(msg):
    global data,first,msg_number_obstacles,costmap_height,costmap_width,start_calculate_flag
    
    rospy.loginfo("in pub_count")

    # while not start_calculate_flag:
    #     rospy.loginfo(start_calculate_flag)
    #     time.sleep(10)
    rospy.loginfo(start_calculate_flag)
    start_calculate_flag = False
    costmap_width = msg.info.width      # 宽度,用于一维数组与矩阵之间的转换
    costmap_height = msg.info.height    # 高度,用于一维数组与矩阵之间的转换
    rospy.loginfo("in")
    data = msg.data

    try:
        ######---准备计算聚类个数
        position_all_DBSCAN = [] # 使用到 DBSCAN 聚类算法中的点集,所有的点
        position_single = []     # 单个像素点位置信息
        number_pix_obstacle = [] # 障碍物中像素点个数
        position_all_keams = []  # 使用到 kmeans 聚类算法中的点集
        # 筛选出障碍物像素点
        for i in range(len(data)):
            if data[i]==100:
                position_single=[i%costmap_width,i/costmap_width]
                position_all_DBSCAN.append(position_single)

        # DBSCAN获得聚类个数，也就是障碍物个数
        X = np.array(position_all_DBSCAN)
        clustering = DBSCAN(eps=5, min_samples=2).fit(X)
        clustering_list = clustering.labels_.tolist()
        number_obstacles = max(clustering_list) # 自动 －1，正好有围墙这个不需要的障碍物
        useful_pix_all = len(data) - clustering_list.count(0)
        rospy.loginfo(number_obstacles)
        rospy.loginfo(clustering_list)
        rospy.loginfo(useful_pix_all)
        # 删除围墙这个障碍物，否则 k-means 无法得出正确分类
        for i in range(len(clustering_list)):
            if clustering_list[i]==0:
                pass
            else:
                position_all_keams.append(position_all_DBSCAN[i])

        # 聚类个数放入k-means算法中进行聚类，计算出障碍物圆心与各类中的像素点个数
        Y = np.array(position_all_keams)
        kmeans = KMeans(n_clusters=number_obstacles, random_state=10).fit(Y)
        clustering_list=kmeans.labels_.tolist()
        # rospy.loginfo(clustering_list)
        distance_kmeans_obstacles = []
        kmeans_centers=kmeans.cluster_centers_.tolist()
        center_x_y_r_n_list = []
        # 创建一个记录障碍物之间距离的list,大小为障碍物个数[[],[],[],[],[]]
        for i in range(number_obstacles):
            distance_kmeans_obstacle = []
            distance_kmeans_obstacles.append(distance_kmeans_obstacle)
        # 计算每个点与其中心点之间的距离[[10,8,6,5,9,7...],[...],[...],[...],[...]]
        for i in range(len(clustering_list)):
            # number_pix_obstacle.append(clustering_list.count(i))
            # 循环一次clustering_list，根据第i点,获取对应点的类别clustering_list[i](这是一个数字,代表第几类,同时也就是列表的第几位.第0类对应第0位)，中心点kmeans_centers[clustering_list[i]],当前这个点的坐标 position_all_keams[i]
            # 计算第clustering_list[i]类的
            # 当前点position_all_keams[i]与clustering_list[i]类的
            # 中心点kmeans_centers[clustering_list[i]]之间的距离
            # 将距离加入到distance_kmeans_obstacles列表中对应的第clustering_list[i]个,
            distance = math.sqrt((position_all_keams[i][0]-kmeans_centers[clustering_list[i]][0])**2+(position_all_keams[i][1]-kmeans_centers[clustering_list[i]][1])**2)
            distance_kmeans_obstacles[clustering_list[i]].append(distance)
        # 将各个类的中心点坐标 x y 以及半径 r ,也就是上一个for循环中得到的各个类中心与其所属点之间距离的最大值 max(distance_kmeans_obstacles[i])
        # 加入到一个列表当中,准备发布topic
        ocupancy_pix_all = 0
        for i in range(len(kmeans_centers)):
            center_x_y_r_n = []
            center_x_y_r_n.append(int(round(kmeans_centers[i][0])))
            center_x_y_r_n.append(int(round(kmeans_centers[i][1])))
            center_x_y_r_n.append(int(max(distance_kmeans_obstacles[i])))
            center_x_y_r_n.append(int(clustering_list.count(i)))
            center_x_y_r_n_list.append(center_x_y_r_n)
            ocupancy_pix_all += int(clustering_list.count(i))


        ###----发布topic
        # msg_number_obstacles = str(number_obstacles) + " - " + str(number_pix_obstacle) + " - " + str(center_x_y_r_n_list)
        msg_number_obstacles = str(number_obstacles) + " - " + str(center_x_y_r_n_list) + " - " + str(1.0*ocupancy_pix_all/useful_pix_all)
        pub_count_obstacles.publish(msg_number_obstacles)
        rospy.loginfo(msg_number_obstacles)
        rospy.loginfo("-------------------------------------------")
        # with open('/home/zlb/msg_number_obstacles111111111111111111111111.txt', 'a+') as f:
        #     f.write(msg_number_obstacles)
        # with open('/home/zlb/costmap_data.txt', 'a+') as f:
        #     f.write("count_obstacle: " + str(a) + "\n" + 
        #     "count_all: " + str(b) + "\n" + 
        #     "ratio: " + str(a*1.0/b) + "\n")
    except expression as identifier:
        pass
   
    

if __name__ == '__main__':
    rospy.init_node('count_obstacles',anonymous=False) 
    rospy.Subscriber('/move_base/global_costmap/costmap',OccupancyGrid,pub_count)
    rospy.Subscriber('/start_calculate',String,start_calculate) # 要随时更新时注释掉这一行,并修改pub_count中的start_calculate_flag
    # rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid,pub_count)
    # rospy.Subscriber('/self_costmap',OccupancyGrid,pub_count)
    # rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,PoseCallBack4)
    pub_count_obstacles = rospy.Publisher('number_obstacles',String , queue_size=1)
    rospy.loginfo("count_obstacles Started!!")
    rospy.spin()