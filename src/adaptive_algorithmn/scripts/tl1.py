#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import time
import math
import sys
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped,Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker

# 什么时候开始？
# 小车开始移动开始？小车接收到目标点开始？
# 当小车接收到目标点信息时计时开始

# 首先得到小车的初始位置 监听 /amcl_pose
# 什么时候结束？
# 当小车与目标点距离小于0.03并且各个速度为0时结束

# 轨迹路径长度如何计算？
# 是要考虑全局规划路线？还是考虑局部规划路线？
# 由于小车在行进过程当中，全局路径规划并不准确，局部规划路径也会发生改变
# 每次刷新小车位置信息时，求刷新时间间隔内的直线距离，累计相加，可以近似为轨迹路径长度

# 距离障碍物的最短距离如何计算？
# 是采取静态地图map? 还是采取实时地图costmap?
# 由于小车在运动过程中对地图的判断是变化的，与静态地图会有误差，所以采用costmap
# 在costmap中的data值被存储为一位数组，其下标与对应的坐标关系为index=(x-ox)/resolution+(y-oy)/resolution*width
# 为获取index值，可以把data放到list列表中，方便直接获取
# 每次刷新都去遍历地图，以像素为单位，两个for循环遍历
# 一旦当data[index]=100时，表示该处有障碍物，求出当前该点与小车的距离
# 与当前最短距离比较，取最小值赋给最小距离，以此进行下去

start_x = 0
start_y = 0
start_x1 = 0
start_y1 = 0
goal_x = 0
goal_y = 0
start_time = 0
stop_time = 0
excution_time = 0.1
cmd_vel_linear_x = 0
cmd_vel_linear_y = 0 
cmd_vel_robot = 0
cmd_vel_angular_x = 0
cmd_vel_angular_y = 0
length_x = 0
length_y = 0
trajectory_smoothness = 0
trajectory_length = 0
angle_robot = 0
angle_vel = 0
angle_first = 0
angle_next = 0
angle_sum = 0
count = 0
flag = True
num = 0
num_list = []
obscale_x = 0
obscale_y = 0
closest_distance = 14
plan_evaluation = 0
pub_cd=None
pub_ts=None
pub_tl=None
pub_et=None
pub_pv=None
flag_first=True
length_direct = 0
data = [0]*40000
resolution = 0.05

def PoseSub():
    #监控话题，并在回调函数中处理
    global pub_cd,pub_et,pub_pv,pub_tl,pub_ts
    rospy.init_node('pose_sub',anonymous=False)
    rospy.Subscriber('/move_base_simple/goal',PoseStamped,PoseCallBack1)
    rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid,PoseCallBack2)
    rospy.Subscriber('/cmd_vel',Twist,PoseCallBack3)
    # rospy.Subscriber('/move_base/global_costmap/costmap',OccupancyGrid,PoseCallBack5)
    rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,PoseCallBack4)
    
    pub_tl = rospy.Publisher('msg_tl', String, queue_size=10)
    pub_ts = rospy.Publisher('msg_ts', String, queue_size=10)
    pub_cd = rospy.Publisher('msg_cd', String, queue_size=10)
    pub_et = rospy.Publisher('msg_et', String, queue_size=10)
    pub_pv = rospy.Publisher('msg_pv', String, queue_size=3)
    rospy.loginfo("Plan Evaluation Started!!")
    rospy.spin()
    # print(closest_distance/(trajectory_length*excution_time+trajectory_smoothness))
    
def PoseCallBack1(msg):
    global goal_x,goal_y,start_time,trajectory_length,count,flag,closest_distance,angle_sum,angle_vel,plan_evaluation,length_direct


    # with open('/home/zlb/performance_auto.txt', 'a+') as f:
    #     f.write(rospy.get_param("/move_base/base_global_planner") + " & " + \
    #     rospy.get_param("/move_base/base_local_planner") + "\n" + str(closest_distance)+"\n" \
    #     + str(trajectory_smoothness) + "\n" + str(trajectory_length) + "\n" + str(excution_time) + \
    #     "\n" + str(plan_evaluation) + "\n ------------ \n")
    # with open('/home/zlb/performance_auto.txt', 'a+') as f:
    #     f.write(rospy.get_param("/move_base/base_global_planner") + " & " + \
    #     rospy.get_param("/move_base/base_local_planner") + "\nplan_evaluation:" + str(plan_evaluation) + "\n ------------ \n")
    #订阅到的目标的坐标信息
    x = msg.pose.position.x
    y = msg.pose.position.y
    goal_x = x
    goal_y = y
    start_time=time.time()
    count=0
    trajectory_length = 0.01
    flag=True
    closest_distance = 14
    angle_sum = 0
    angle_vel = 0
    flag_first=True
    length_direct = 0
    
def PoseCallBack2(msg):
    global num,num_list
    
    #订阅到的地图的data信息
    num = msg.data
    num_list = list(num)


def PoseCallBack3(msg):
    global cmd_vel_linear_x,cmd_vel_linear_y,cmd_vel_angular_x,cmd_vel_angular_y,cmd_vel_robot,angle_robot,angle_vel,angle_next
    
    #订阅到的小车的速度信息
    cmd_vel_linear_x = msg.linear.x
    cmd_vel_linear_x = msg.linear.y
    cmd_vel_angular_x = msg.angular.x
    cmd_vel_angular_y = msg.angular.y
    
    # 计算速度之间的角度angle
    # cmd_vel = (math.sqrt)(cmd_vel_linear_x**2+cmd_vel_linear_y**2)
    # if(cmd_vel!=0):
    #     angle_first = (math.acos)(cmd_vel_linear_x/cmd_vel)
    #     angle_vel += (angle_first-angle_next)**2
    #     angle_next = angle_first


def PoseCallBack4(msg):
    global start_x,start_y,start_x1,start_y1,start_time,stop_time
    global excution_time,cmd_vel_linear_x,cmd_vel_linear_y,cmd_vel_angular_x,cmd_vel_angular_y,length_x,length_y
    global trajectory_smoothness,trajectory_length,angle_robot,angle_first,angle_first1,angle_sum,count
    global num_list,obscale_x,obscale_y,closest_distance
    global pub_cd,pub_et,pub_pv,pub_tl,pub_ts,flag,flag_first,plan_evaluation,length_direct
    global n,data,angle_vel,angle_next
    count += 1

    #订阅到的小车的坐标信息
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    i = x/resolution
    j = y/resolution
    # rospy.loginfo((i,j))
    # rospy.loginfo(len(data))
    # data[int(i*200+j)] = 2
    
    # with open('/home/zlb/桌面/map.txt', 'w') as f: 
    #     f.write(str(data))

    if flag:
        start_x = x
        start_y = y
        flag = False


    # 获取小车与目标点的路径长度（局部规划路径长度）
    # if(cmd_vel_linear_x != 0 and cmd_vel_linear_y != 0 and cmd_vel_angular_x != 0 and cmd_vel_angular_y != 0):
    length_x = x-start_x1
    length_y = y-start_y1
    length = (math.sqrt)(length_x**2+length_y**2)
    length_direct = (math.sqrt)((goal_x-start_x)**2+(goal_y-start_y)**2)
    rospy.loginfo("start_x:"+str(start_x))
    rospy.loginfo("--------length_direct:"+str(length_direct))
    length2goal=((math.sqrt)((x-goal_x)**2+(y-goal_y)**2))
    cmd_vel = (math.sqrt)(cmd_vel_linear_x**2+cmd_vel_linear_y**2)
    if(cmd_vel!=0 and length2goal>=1):
        angle_first = (math.acos)(cmd_vel_linear_x/cmd_vel)
        angle_vel += (angle_first-angle_next)**2
        angle_next = angle_first
    if(length != 0 and length2goal>=1):
        angle_first1 = (math.acos)(length_x/length)
        angle_robot = angle_first1-angle_robot
        angle_first1 = angle_robot
        angle_sum += abs(angle_robot)
        trajectory_smoothness = (0.5*angle_vel+0.5*angle_sum)/count
    rospy.loginfo("trajectory_smoothness:"+str(trajectory_smoothness))
    #########################################################################
    msg_ts = "Trajectory Smoothness: " + str(trajectory_smoothness)
    pub_ts.publish(msg_ts)
    #########################################################################
    
    trajectory_length += (math.sqrt)(length_x**2+length_y**2)
    rospy.loginfo("trajectory_length:"+str(trajectory_length))
    start_x1 = x
    start_y1 = y
    #########################################################################
    msg_tl = "Trajectory Length: " + str(trajectory_length)
    pub_tl.publish(msg_tl)
    #########################################################################

    # 小车移动到目标点附近结束(是否需要同时判断 /cmd_vel 为零，更精确)
    rospy.loginfo("--------------x------------"+str(x))
    rospy.loginfo("--------------y------------"+str(y))
    rospy.loginfo("--------------goal_x------------"+str(goal_x))
    rospy.loginfo("--------------goal_y------------"+str(goal_y))
    
    rospy.loginfo("--------------------------"+str(length2goal))

    rospy.loginfo(flag_first)
    if not flag_first:
        stop_time = time.time()
        excution_time = stop_time - start_time
        plan_evaluation = 10000*closest_distance/(trajectory_length*excution_time*trajectory_smoothness/length_direct)
        rospy.loginfo("excution_time:"+str(excution_time))
        rospy.loginfo("Plan Evaluation: " + str(plan_evaluation))
        #########################################################################
        msg_et = "Excution Time: " + str(excution_time)
        pub_et.publish(msg_et)
        msg_pv = "Plan Evaluation: " + str(plan_evaluation)
        pub_pv.publish(msg_pv)
        #########################################################################
        
        # print(closest_distance/(trajectory_length*excution_time+trajectory_smoothness))
    flag_first = False
    #小车距离障碍物的最近距离
    for obscale_x in range(0,60):
        for obscale_y in range(0,60):
            if(len(num_list) != 0 and num_list[(int(obscale_x+obscale_y*60))] == 100):
                if(closest_distance > (math.sqrt)(((30-obscale_x)*0.05)**2+(((30-obscale_y)*0.05)**2))):
                    closest_distance = (math.sqrt)(((30-obscale_x)*0.05)**2+(((30-obscale_y)*0.05)**2))
    rospy.loginfo("closest_distance:"+str(closest_distance))
    #########################################################################
    msg_cd = "Closest Distance: " + str(closest_distance)
    pub_cd.publish(msg_cd)
    #########################################################################


if __name__ == '__main__':
    PoseSub()