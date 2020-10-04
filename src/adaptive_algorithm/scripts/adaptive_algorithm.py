#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy,os
import math
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from std_msgs.msg import String

'''
author:zlb
实现的功能:
    节点名称:pub_self_costmap 
    订阅:/move_base/global_costmap/costmap等OccupancyGrid 数据格式的话题消息
        /amcl_pose,根据其位置信息结合订阅到的 costmap 发布 基于自身位置的self_costmap,OccupancyGrid 数据格式
        /number_obstacles 获取障碍物基本信息,障碍物个数/中心位置 半径 实际像素点个数,
        "7 - [[82, 21, 10, 317], [131, 124, 10, 317], [44, 142, 10, 317], [38, 52, 10, 317], [165, 145, 10, 317], [30, 106, 10, 317], [94, 115, 17, 740]]"
        /risk_obstacles   障碍物威胁程度
    发布:/self_costmap, OccupancyGrid, 发布的 self_costmap 就是探测的范围,也就是需要进行障碍物信息计算的区域

发布要探测的区域,接受探测完成的信息,根据障碍物信息选择算法,监听到risk_obstacles时,才会进行算法修改.同时发布下一个目标点!!!这一步需要商榷,相当于每次发布一个局部的目标点,然后最后到达最终的目标点
'''


data = []

msg_costmap = OccupancyGrid() # 这个就是 self_costmap
min_pix_x = 0   # self_costmap的四个角的信息
min_pix_y = 0
max_pix_x = 0
max_pix_y = 0
meter_width = 6 # self_costmap的米边长
obstacle_threshold = 100 # 像素点被认定为是一个障碍物的像素阈值
resolution = 0.05 # 代价地图的分辨率,都一样
glb_costmap_shape = 200 # 全局代价地图的像素边长

number_obstacles = 0 # 障碍物个数
count_obstacle_pix=0 # 障碍物像素点个数
risk_obstacles = 0  # 障碍物威胁程度

def write(msg):
    # 得到全局代价地图作为 self_costmap 的基础
    global data,first,obstacle_threshold

    rospy.loginfo("in")
    data = msg.data
    rospy.loginfo(len(data))
    a = data.count(obstacle_threshold)
    b = len(data)
    # with open('/home/zlb/costmap.txt', 'a+') as f:
    #     f.write(str(msg))
    with open('/home/zlb/costmap_data.txt', 'a+') as f:
        f.write("count_obstacle: " + str(a) + "\n" + 
        "count_all: " + str(b) + "\n" + 
        "ratio: " + str(a*1.0/b) + "\n")


def pub_self_costmap(msg):
    global data,msg_costmap,resolution,glb_costmap_shape,meter_width,obstacle_threshold,count_obstacle_pix,len_self_costmap_data,pub_costmap

    #订阅到的小车的坐标信息
    x = msg.pose.pose.position.x  # 米单位下的位置
    y = msg.pose.pose.position.y
    pix_x = int(round(x/resolution)) # 像素单位下的位置
    pix_y = int(round(y/resolution))
    pix_width = int(meter_width/resolution) # 像素单位下的边长

    #图像坐标正中心是小车,当图像的为(min_pix_x,min_pix_y),右上角为(max_pix_x,max_pix_y)
    if pix_x > (pix_width/2):
        min_pix_x = pix_x - (pix_width/2)
        if pix_x < glb_costmap_shape-(pix_width/2):
            max_pix_x = pix_x + (pix_width/2)
        else:
            max_pix_x = glb_costmap_shape
    else:
        max_pix_x = pix_x + (pix_width/2)
        min_pix_x = 0

    #当pix_y大于像素完整self_costmap的边长的一半时说明下界即为 min_pix_y = pix_y - (pix_width/2) 
    #但是上界还不确定,还需要继续判断,当上界未超出glb_costmap_shape,上界为max_pix_y = pix_y + (pix_width/2)
    #当上界未超出glb_costmap_shape,上界为max_pix_y = glb_costmap_shape
    if pix_y > (pix_width/2): 
        min_pix_y = pix_y - (pix_width/2)     
        if pix_y < glb_costmap_shape-(pix_width/2):
            max_pix_y = pix_y + (pix_width/2)
        else:
            max_pix_y = glb_costmap_shape
    #当pix_y小于像素完整self_costmap的边长的一半时说明下界即为 min_pix_y = 0
    #此时上界肯定为  max_pix_y = pix_y + (pix_width/2)
    else:
        max_pix_y = pix_y + (pix_width/2)
        min_pix_y = 0

    rospy.loginfo("min_pix_x = " + str(min_pix_x))
    rospy.loginfo("min_pix_y = " + str(min_pix_y))
    rospy.loginfo("max_pix_x = " + str(max_pix_x))
    rospy.loginfo("max_pix_y = " + str(max_pix_y))

    # 实际展示出来的自定义的代价地图的长宽以及数据
    self_costmap_width = int(max_pix_x-min_pix_x)
    self_costmap_heigth = int(max_pix_y-min_pix_y)
    self_costmap_data = []

    # 准备topic:/self_costmap 的信息
    msg_costmap.header.stamp = rospy.Time.now()
    msg_costmap.header.frame_id = "odom"
    
    msg_costmap.info.resolution = resolution

    msg_costmap.info.width=self_costmap_width
    msg_costmap.info.height=self_costmap_heigth
    
    # 设置代价地图 self_costmap 的边界,使得代价地图展现出来的部分都在地图中,x y 都要设置
    if min_pix_x>0: 
        msg_costmap.info.origin.position.x=int(min_pix_x*resolution)
    else:
        msg_costmap.info.origin.position.x=0
    if min_pix_y>0:
        msg_costmap.info.origin.position.y=int(min_pix_y*resolution)
    else:
        msg_costmap.info.origin.position.y=0
    msg_costmap.info.origin.position.z=0.0
    
    # 相对于 self_costmap 源点的方向
    msg_costmap.info.origin.orientation.x=0.0
    msg_costmap.info.origin.orientation.y=0.0
    msg_costmap.info.origin.orientation.z=0.0
    msg_costmap.info.origin.orientation.w=1.0
    
    # 更新地图
    for i in range(min_pix_y,max_pix_y):
        x_start = int(min_pix_x+i*glb_costmap_shape)
        x_end = int(max_pix_x+i*glb_costmap_shape)
        data_temp = list(data[x_start:x_end])
        self_costmap_data = self_costmap_data+data_temp

    # 计算 self_costmap_data 中像素的个数 len_self_costmap_data
    len_self_costmap_data = len(self_costmap_data)
    # 像素值达到100 也就是认定为障碍物的像素点个数
    count_obstacle_pix = self_costmap_data.count(obstacle_threshold)
    rospy.loginfo("----------len_self_costmap_data-----------"+str(len_self_costmap_data))
    rospy.loginfo("------------count_obstacle_pix------------"+str(count_obstacle_pix))
    
    ##########----------------------------------以下写入方法switch_algorithmn()，调用之----------------------------------
    switch_algorithmn()
    ##########----------------------------------以上写入方法switch_algorithmn()，调用之----------------------------------
    #填充self_costmap_data并发布
    msg_costmap.data = self_costmap_data
    pub_costmap.publish(msg_costmap)
    # with open('/home/zlb/costmap_1.txt', 'a+') as f:
    #     f.write(str(self_costmap_data))

def switch_algorithmn():
    # 根据得到的 障碍物数
    global count_obstacle_pix,len_self_costmap_data,base_global_planner
    rospy.loginfo("--------------------------into switch_algorithmn----------------------------------")
    base_global_planner = "" # 算法
    base_local_planner = ""
    rospy.loginfo(1.0*count_obstacle_pix/len_self_costmap_data)
    if 1.0*count_obstacle_pix/len_self_costmap_data < 0.10:
        base_global_planner = "global_planner/GlobalPlanner"
        base_local_planner = "base_local_planner/TrajectoryPlannerROS"
        rospy.loginfo("global_planner/GlobalPlanner")
    else:
        base_global_planner = "navfn/NavfnROS"
        base_local_planner = "dwa_local_planner/DWAPlannerROS"
        rospy.loginfo("navfn/NavfnROS")
    # 判断算法组合是否修改,如果修改了,就对move_base进行动态调参
    if base_global_planner != rospy.get_param("/move_base/base_global_planner"):
        sh_command = "rosrun dynamic_reconfigure dynparam set /move_base '{'base_global_planner': " + base_global_planner +", 'conservative_reset_dist': 3.0, 'groups': {'base_global_planner': "+ base_global_planner + ", 'planner_frequency': 5.0, 'parent': 0, 'conservative_reset_dist': 3.0, 'shutdown_costmaps': False, 'restore_defaults': False, 'groups': {}, 'oscillation_timeout': 10.0, 'id': 0, 'controller_patience': 15.0, 'name': 'Default', 'parameters': {}, 'type': '', 'clearing_rotation_allowed': True, 'state': True, 'oscillation_distance': 0.2, 'max_planning_retries': -1, 'base_local_planner': " + base_local_planner + " , 'recovery_behavior_enabled': True, 'planner_patience': 5.0, 'controller_frequency': 10.0}, 'controller_patience': 15.0, 'max_planning_retries': -1, 'shutdown_costmaps': False, 'clearing_rotation_allowed': True, 'restore_defaults': False, 'oscillation_distance': 0.2, 'planner_frequency': 5.0, 'oscillation_timeout': 10.0, 'base_local_planner': " + base_local_planner + "',recovery_behavior_enabled': True, 'planner_patience': 5.0, 'controller_frequency': 10.0}'"
        os.system(sh_command) 

def pub_self_costmap_temp(msg):
    global data,msg_costmap,resolution,glb_costmap_shape,meter_width,obstacle_threshold,pub_costmap

    # 准备topic:/self_costmap 的信息
    msg_costmap.header.stamp = rospy.Time.now()
    msg_costmap.header.frame_id = "map"
    
    msg_costmap.info.resolution = 0.05

    msg_costmap.info.width=200
    msg_costmap.info.height=200
    
    msg_costmap.info.origin.position.x=0
    msg_costmap.info.origin.position.y=0
    msg_costmap.info.origin.position.z=0.0
    
    # 相对于 self_costmap 源点的方向
    msg_costmap.info.origin.orientation.x=0.0
    msg_costmap.info.origin.orientation.y=0.0
    msg_costmap.info.origin.orientation.z=0.0
    msg_costmap.info.origin.orientation.w=1.0
    msg_costmap.data = data

    pub_costmap.publish(msg_costmap)
    rospy.loginfo("----msg_costmap----")
    # with open('/home/zlb/costmap.txt', 'a+') as f:
    #     f.write(str(msg_costmap.data))

def sub_number_obstacles(msg):
    global number_obstacles,count_obstacle_pix

    data_list = msg.data.split(" - ")

    ## 获得障碍物数量
    number_obstacles = int(data_list[0])
    center_x_y_r_list = eval(data_list[1])
    rospy.loginfo(str(center_x_y_r_list) + "-----\n")




    # ## 获得障碍物数量
    # number_obstacles = int(data_list[0])
    # count_obstacle_pix_list = eval(data_list[1])
    # count_obstacle_center_list = eval(data_list[2])
    
    # center_x_y_r_list = []
    # ## 获得障碍物的像素点个数以计算覆盖率，除围墙以外
    # for i in range(number_obstacles):
    #     center_x_y_r = []
    #     count_obstacle_pix += int(count_obstacle_pix_list[i])
    #     center_x_y_r.append(round(count_obstacle_center_list[i][0]))
    #     center_x_y_r.append(round(count_obstacle_center_list[i][1]))
    #     center_x_y_r.append(round(math.sqrt(int(count_obstacle_pix_list[i])/3.14)))
    #     center_x_y_r_list.append(center_x_y_r)
    # rospy.loginfo(str(center_x_y_r_list) + "-----\n")

    # 计算各个障碍物之间的距离,接收到的数据里有center_x_y_r,已知坐标以及半径的两个圆之间计算距离.
    distance_each_other_list = []   
    for i in range(len(center_x_y_r_list)):
        length = i + 1
        while length < len(center_x_y_r_list):
            distance_each_other_single = []
            distance_each_other_single.append(i)
            distance_each_other_single.append(length)
            distance_each_other_single.append(int(abs(
                math.sqrt((center_x_y_r_list[length][0]-center_x_y_r_list[i][0])**2+(center_x_y_r_list[length][1]-center_x_y_r_list[i][1])**2)
                -center_x_y_r_list[length][2]-center_x_y_r_list[i][2])))
            distance_each_other_list.append(distance_each_other_single)
            length += 1
    rospy.loginfo(str(distance_each_other_list) + "-----\n")
def sub_risk_obstacles(msg):
    global risk_obstacles
    risk_obstacles = eval(msg.data)
    ###调用switch_algorithmn
if __name__ == '__main__':
    rospy.init_node('cost_write',anonymous=False) 
    # rospy.init_node('pub_self_costmap',anonymous=False) 
    rospy.Subscriber('/move_base/global_costmap/costmap',OccupancyGrid,write)
    # rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid,write)
    rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,pub_self_costmap)
    # rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,pub_self_costmap_temp)
    rospy.Subscriber('/number_obstacles',String,sub_number_obstacles)
    rospy.Subscriber('/risk_obstacles',String,sub_risk_obstacles)
    pub_costmap = rospy.Publisher('self_costmap', OccupancyGrid, queue_size=10)
    rospy.loginfo("adaptive node Started!!")
    rospy.spin()

    