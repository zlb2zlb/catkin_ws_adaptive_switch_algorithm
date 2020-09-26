#!/usr/bin/python3
# -*- coding: utf-8 -*-
'''
author:zlb
实现的功能:
    节点名称:calculate_risk_obstacle
    订阅:/move_base_simple/goal,PoseStamped,PoseCallBack1,    订阅当前目标点信息
        /number_obstacles,String,sub_number_obstacles,       订阅当前区域障碍物信息  
        /amcl_pose,PoseWithCovarianceStamped,PoseCallBack4,  订阅当前位姿信息
    发布:/risk_obstacles,String,                              发布当前位姿下接下来一段地图的障碍物威胁程度之和
'''
import math
import rospy
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from std_msgs.msg import String

# from tf.transformations import euler_from_quanternion
import tf2_ros
import tf_quaternion
from tf.transformations import euler_from_quaternion
import numpy as np

msg_number_obstacles = "7 - [[82, 21, 10, 317], [131, 124, 10, 317], \
[44, 142, 10, 317], [38, 52, 10, 317], [165, 145, 10, 317], [30, 106, 10, 317], [94, 115, 17, 740]]"

data_list = msg_number_obstacles.split(" - ")

## 获得障碍物数量
center_x_y_r_n_list = eval(data_list[1])

risk_obstacle=1000

vector_car_center = None
vector_car_orientation = None 

risk_obstacle_all = 0

# 小车坐标信息和目标点坐标信息
x_position=None
y_position=None
goal_x_position = 1000
goal_y_position = 1000
goal_x_orientation,goal_y_orientation,goal_z_orientation,goal_w_orientation = 0,0,0,1
def PoseCallBack4(msg):
    global data_list,center_x_y_r_n_list
    global risk_obstacle,risk_obstacle_all,vector_car_center,vector_car_orientation
    global goal_x_position,goal_y_position,x_position,y_position,goal_x_orientation,goal_y_orientation,goal_z_orientation,goal_w_orientation

    risk_obstacle=0
    #订阅到的小车的坐标信息
    x_position = msg.pose.pose.position.x  # 米单位下的位置
    y_position = msg.pose.pose.position.y
    x_orientation = msg.pose.pose.orientation.x
    y_orientation = msg.pose.pose.orientation.y
    z_orientation = msg.pose.pose.orientation.z
    w_orientation = msg.pose.pose.orientation.w

    # 判断小车是否到达目标点附近,到达目标点以后开始计算方向向量
    if math.sqrt((x_position-goal_x_position)**2+(y_position-goal_y_position)**2)<0.1:
        # 小车的 四元数--->欧拉角--->xy向量
        quanternion = (
            x_orientation,
            y_orientation,
            z_orientation,
            w_orientation
        )
        euler = euler_from_quaternion(quanternion)
        rospy.loginfo(euler)
        x_euler = math.cos(euler[2])*math.cos(euler[1])
        y_euler = math.sin(euler[2])*math.cos(euler[1])

        # 目标点的 四元数--->欧拉角--->xy向量
        quanternion_1 = (
            goal_x_orientation,
            goal_y_orientation,
            goal_z_orientation,
            goal_w_orientation
        )
        euler_1 = euler_from_quaternion(quanternion_1)
        rospy.loginfo(euler_1)
        x_euler_1 = math.cos(euler_1[2])*math.cos(euler_1[1])
        y_euler_1 = math.sin(euler_1[2])*math.cos(euler_1[1])
        
        # 小车的方向向量同目标点的方向向量转化为矩阵
        vector_car_goal=np.array([x_euler_1,y_euler_1])
        vector_car_orientation=np.array([x_euler,y_euler])

        # 计算 小车的方向向量 同 目标点的方向向量 两个向量之间的 夹角,并转换为角度
        Lvector_car_goal=np.sqrt(vector_car_goal.dot(vector_car_goal))
        Lvector_car_orientation=np.sqrt(vector_car_orientation.dot(vector_car_orientation))
        cos_angle=vector_car_goal.dot(vector_car_orientation)/(Lvector_car_goal*Lvector_car_orientation)
        angle=np.arccos(cos_angle)
        angle2=angle*360/2/np.pi
        rospy.loginfo("***/**/*/*/*/*/* %s",str(angle2))

        # 当两个向量之间的夹角已经小于某个阈值之后,判定小车已经到达目标点的目标方向,开始计算接下来一段路程的威胁程度
        if angle2 < 10:
            # 计算威胁程度,x_y_r_n = [82, 21, 10, 317]
            for x_y_r_n in center_x_y_r_n_list:
                # 障碍物中心点与小车当前位置之间的向量 vector_car_center 以及 小车的朝向向量 vector_car_orientation
                vector_car_center=np.array([x_y_r_n[0]-x_position/0.05,x_y_r_n[1]-y_position/0.05])
                vector_car_orientation=np.array([x_euler,y_euler])
                # 计算两个向量之间的夹角,并转换为角度
                Lvector_car_center=np.sqrt(vector_car_center.dot(vector_car_center))
                Lvector_car_orientation=np.sqrt(vector_car_orientation.dot(vector_car_orientation))
                cos_angle=vector_car_center.dot(vector_car_orientation)/(Lvector_car_center*Lvector_car_orientation)
                angle=np.arccos(cos_angle)
                angle2=angle*360/2/np.pi

                rospy.loginfo("angle2---%s--",angle2)
                # 如果夹角小于90度,说明该障碍物会出现在小车前进道路范围内,是具有威胁的,如果大于90度,则是没有威胁的
                if angle2<90:
                    risk_obstacle = (1-angle2/90)/(math.sqrt((x_y_r_n[0]-x_position/0.05)**2+(x_y_r_n[1]-y_position/0.05)**2))*200
                else:
                    risk_obstacle = 0
                # 将所有的威胁都加起来
                risk_obstacle_all += risk_obstacle 
                rospy.loginfo("risk_obstacle--%s--%s--%s",str(x_y_r_n),str(risk_obstacle),str(risk_obstacle_all))
            
            pub_risk_obstacles.publish(str(risk_obstacle_all))
            rospy.loginfo("---------------%s-----------------",str(risk_obstacle_all))
            risk_obstacle = 0
            risk_obstacle_all = 0

    # rospy.loginfo((x_orientation,y_orientation,z_orientation,w_orientation,(w_orientation**2+x_orientation**2-y_orientation**2-z_orientation**2)))
    # rospy.loginfo(2*(x_orientation*y_orientation-z_orientation*w_orientation))
    # rospy.loginfo((w_orientation**2+x_orientation**2-y_orientation**2-z_orientation**2))
    
    # angle = math.atan(2*(x_orientation*y_orientation-z_orientation*w_orientation)/(w_orientation**2+x_orientation**2-y_orientation**2-z_orientation**2))/3.14*180

    # rospy.loginfo("angle:" + str(angle))
def PoseCallBack1(msg):
    global goal_x_position,goal_y_position,goal_x_orientation,goal_y_orientation,goal_z_orientation,goal_w_orientation

    #订阅到的目标的位姿信息
    goal_x_position = msg.pose.position.x
    goal_y_position = msg.pose.position.y
    goal_x_orientation = msg.pose.orientation.x
    goal_y_orientation = msg.pose.orientation.y
    goal_z_orientation = msg.pose.orientation.z
    goal_w_orientation = msg.pose.orientation.w


   

def sub_number_obstacles(msg):
    global number_obstacles,count_obstacle_pix,center_x_y_r_n_list

    # 负责不断传入下一个costmap区域的障碍物信息
    data_list = msg.data.split(" - ")
    center_x_y_r_n_list = eval(data_list[1])

    rospy.loginfo(str(center_x_y_r_n_list) + "-----\n")

    ######################### 静态起始点和静态目标点 #########################
    ## 获得障碍物数量
    risk_obstacle_all=0

    #订阅到的小车的坐标信息
    x_position = 1  # 米单位下的位置
    y_position = 1
    x_orientation = 0
    y_orientation = 0
    z_orientation = 0
    w_orientation = 1
    # 小车的 四元数--->欧拉角--->xy向量
    quanternion = (
        x_orientation,
        y_orientation,
        z_orientation,
        w_orientation
    )
    euler = euler_from_quaternion(quanternion)
    print(euler)
    x_euler = math.cos(euler[2])*math.cos(euler[1])
    y_euler = math.sin(euler[2])*math.cos(euler[1])


    for x_y_r_n in center_x_y_r_n_list:
        # 障碍物中心点与小车当前位置之间的向量 vector_car_center 以及 小车的朝向向量 vector_car_orientation
        vector_car_center=np.array([x_y_r_n[0]-x_position/0.05,x_y_r_n[1]-y_position/0.05])
        vector_car_orientation=np.array([x_euler,y_euler])
        # 计算两个向量之间的夹角,并转换为角度
        Lvector_car_center=np.sqrt(vector_car_center.dot(vector_car_center))
        Lvector_car_orientation=np.sqrt(vector_car_orientation.dot(vector_car_orientation))
        cos_angle=vector_car_center.dot(vector_car_orientation)/(Lvector_car_center*Lvector_car_orientation)
        angle=np.arccos(cos_angle)
        angle2=angle*360/2/np.pi

        # 如果夹角小于90度,说明该障碍物会出现在小车前进道路范围内,是具有威胁的,如果大于90度,则是没有威胁的
        if angle2<90:
            risk_obstacle = (1-angle2/90)/(math.sqrt((x_y_r_n[0]-x_position/0.05)**2+(x_y_r_n[1]-y_position/0.05)**2))*200
        else:
            risk_obstacle = 0
        # 将所有的威胁都加起来
        risk_obstacle_all += risk_obstacle 
        rospy.loginfo("risk_obstacle--%s--%s--%s",str(x_y_r_n),str(risk_obstacle),str(risk_obstacle_all))
    rospy.loginfo("risk_obstacle--%s--%s--%s",str(x_y_r_n),str(risk_obstacle),str(risk_obstacle_all))
    pub_risk_obstacles.publish(str(risk_obstacle_all))
        ######################### 静态起始点和静态目标点 #########################

if __name__ == '__main__':
    rospy.init_node('calculate_risk_obstacle',anonymous=False) 
    # rospy.Subscriber('/move_base_simple/goal',PoseStamped,PoseCallBack1)
    # 暂时需要的是固定的对于地图中 (xyz,xyzw),(1,1,0,0,0,0,1)这个点的威胁程度,暂时不需要订阅 /amcl_pose
    # rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,PoseCallBack4)
    rospy.Subscriber('/number_obstacles',String,sub_number_obstacles)
    pub_risk_obstacles = rospy.Publisher('risk_obstacles',String, queue_size=1)
    rospy.loginfo("calculate_risk_obstacle Started!!")
    rospy.spin()

