#!/usr/bin/python3
# -*- coding: utf-8 -*-
'''
author:zlb
实现的功能:
    节点名称:auto_plan_evaluation
    订阅:/msg_tl, String, AchieveGoal_tl,  评价函数
        /msg_cd, String, AchieveGoal_cd,  评价函数  
        /msg_pv,String,AchieveGoal_pv,  评价函数
        /msg_et, String, 执行时长
        /amcl_pose,PoseWithCovarianceStamped,AmclStartGoal,  订阅当前位姿信息
    发布:/move_base_simple/goal, PoseStamped, 发布目标点位姿信息
        /initialpose, PoseWithCovarianceStamped, 初始化位姿信息 
        /move_base/cancel, GoalID, 取消前往目标点的任务
'''
import math
import os
import rospy
import rospkg
import roslaunch
import time
import rosnode
from std_msgs.msg import String
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped,PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID
from dynamic_reconfigure.parameter_generator_catkin import *

msg_initialpose = PoseWithCovarianceStamped()
# pose部分,包括 坐标position 和 方向orientation
msg_initialpose.pose.pose.position.x = 1
msg_initialpose.pose.pose.position.y = 1
msg_initialpose.pose.pose.position.z = 0
msg_initialpose.pose.pose.orientation.x = 0
msg_initialpose.pose.pose.orientation.y = 0
msg_initialpose.pose.pose.orientation.z = 0
msg_initialpose.pose.pose.orientation.w = 1
msg_initialpose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, \
  0.0, 0.25, 0.0, 0.0, 0.0, 0.0, \
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
  0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]


# 发布目标位置
# header部分
msg_goal = PoseStamped()
# pose部分,包括 坐标position 和 方向orientation
msg_goal.pose.position.x = 9
msg_goal.pose.position.y = 9
msg_goal.pose.position.z = 0
msg_goal.pose.orientation.x = 0
msg_goal.pose.orientation.y = 0
msg_goal.pose.orientation.z = 0
msg_goal.pose.orientation.w = 1

# 发布起始点位置
# header部分
msg_start = PoseStamped()
# pose部分,包括 坐标position 和 方向orientation
msg_start.pose.position.x = 1
msg_start.pose.position.y = 1
msg_start.pose.position.z = 0
msg_start.pose.orientation.x = 0
msg_start.pose.orientation.y = 0
msg_start.pose.orientation.z = 0
msg_start.pose.orientation.w = 1

msg_cancel = GoalID()
msg_cancel.id = ''

achieve_start_flag = False # 是否回到原点
achieve_goal_flag = False # 是否到达终点
got_pv_flag = False 

start_x = 1 # 起始点的坐标
start_y = 1
goal_x = 9 # 目标点的坐标
goal_y = 9
world_number_start =80# 第几张图开始
world_number_end = 100# 第几张图结束

cylinder_number = [16] # 有的障碍物数量
# # bgp_all = ["global_planner/GlobalPlanner","navfn/NavfnROS"] # 全局规划算法的集合
blp_all = ["eband_local_planner/EBandPlannerROS","dwa_local_planner/DWAPlannerROS","base_local_planner/TrajectoryPlannerROS"] # 局部....
bgp_all = ["global_planner/GlobalPlanner"] # 全局规划算法的集合
# blp_all = ["dwa_local_planner/DWAPlannerROS"] # 局部....
# blp_all = ["dwa_local_planner/DWAPlannerROS"]

all_combine_average_value = {
    'global_planner/GlobalPlanner + eband_local_planner/EBandPlannerROS':{
        'sum':0,'success_count':0,'fail_count':0
    },
    'global_planner/GlobalPlanner + dwa_local_planner/DWAPlannerROS':{
        'sum':0,'success_count':0,'fail_count':0
    },
    'global_planner/GlobalPlanner + base_local_planner/TrajectoryPlannerROS':{
        'sum':0,'success_count':0,'fail_count':0
    },
    'navfn/NavfnROS + eband_local_planner/EBandPlannerROS':{
        'sum':0,'success_count':0,'fail_count':0
    },
    'navfn/NavfnROS + dwa_local_planner/DWAPlannerROS':{
        'sum':0,'success_count':0,'fail_count':0
    },
    'navfn/NavfnROS + base_local_planner/TrajectoryPlannerROS':{
        'sum':0,'success_count':0,'fail_count':0
    }}
pv_best = {"algorithmn_combine":"base","plan_evaluation":0} # 字典形式的
file_name = '/home/zlb/dev/ros_collections/catkin_ws_adaptive_switch_algorithm/src/adaptive_algorithm/20201021_adaptive'
save_dir = '/home/zlb/dev/ros_collections/catkin_ws_adaptive_switch_algorithm/src/adaptive_algorithm/maps_1/slim_cylinder_3/world_cylinder_'
algorithmn_combine = "" # 单次规划中的算法组合 初始化空字符串
algorithmn_combine_best = "" # 单个环境中的最优算法组合 初始化空字符串
plan_evaluation = 0 # 评价函数
plan_evaluation_max = 0 # 最高评价函数
trajectory_length = 0
closest_distance = 0
et = 0  # 执行时长，小车接受到命令开始到停止运动结束
cost_time = 0 ## 执行时长，从程序发出命令开始到接收到小车运动停止结束
base_global_planner = "navfn/NavfnROS" # 默认为 "navfn/NavfnROS"
base_local_planner = "base_local_planner/TrajectoryPlannerROS" # 默认为 "base_local_planner/TrajectoryPlannerROS"

# 特征 障碍物个数 障碍物实际占据像素点个数 障碍物威胁程度
number_obstacles = 0
distance_each_other_obstacles_average = 0
risk_obstacles = 0
ocupancy_obstacles = 0

# 动态调参 调整全局和局部参数
# msg_algorithmn_combine = "rosrun dynamic_reconfigure dynparam set /move_base '{'base_global_planner': "\
#  + base_global_planner +", 'conservative_reset_dist': 3.0, 'groups': {'base_global_planner': "\
#  + base_global_planner + ", 'planner_frequency': 5.0, 'parent': 0, 'conservative_reset_dist': 3.0, 'shutdown_costmaps': False, 'restore_defaults': False, 'groups': {}, 'oscillation_timeout': 10.0, 'id': 0, 'controller_patience': 15.0, 'name': 'Default', 'parameters': {}, 'type': '', 'clearing_rotation_allowed': True, 'state': True, 'oscillation_distance': 0.2, 'max_planning_retries': -1, 'base_local_planner': " \
#  + base_local_planner + " , 'recovery_behavior_enabled': True, 'planner_patience': 5.0, 'controller_frequency': 10.0}, 'controller_patience': 15.0, 'max_planning_retries': -1, 'shutdown_costmaps': False, 'clearing_rotation_allowed': True, 'restore_defaults': False, 'oscillation_distance': 0.2, 'planner_frequency': 5.0, 'oscillation_timeout': 10.0, 'base_local_planner': " \
#  + base_local_planner + "',recovery_behavior_enabled': True, 'planner_patience': 5.0, 'controller_frequency': 10.0}'"

turtlebot3_world_launch = "/home/zlb/dev/ros_collections/catkin_ws_adaptive_switch_algorithm/src/adaptive_algorithm/launch/turtlebot3_world.launch"
turtlebot3_navigation_launch = "/home/zlb/dev/ros_collections/catkin_ws_adaptive_switch_algorithm/src/adaptive_algorithm/launch/turtlebot3_navigation.launch"



def main():
    global msg_algorithmn_combine,msg_gazebo,msg_goal,msg_initialpose,msg_rviz,msg_start
    global achieve_start_flag,achieve_goal_flag,got_pv_flag
    global start_x,start_y,goal_x,goal_y
    global bgp_all,blp_all,world_number,cylinder_number
    global pv_best,algorithmn_combine,algorithmn_combine_best,plan_evaluation,plan_evaluation_max,trajectory_length,closest_distance
    global base_global_planner,base_local_planner
    global msg_initialpose,msg_goal,file_name,save_dir,turtlebot3_world_launch,turtlebot3_navigation_launch
    global number_obstacles,distance_each_other_obstacles_average,ocupancy_obstacles,risk_obstacles,et,cost_time

    rospy.init_node('auto_plan_evaluation',anonymous=False)  
    
    # rospy.Subscriber('/msg_tl', String, AchieveGoal_tl)
    # rospy.Subscriber('/msg_cd', String, AchieveGoal_cd)      
    rospy.Subscriber('/msg_pv',String,AchieveGoal_pv)
    # rospy.Subscriber('/msg_ts', String, AchieveGoal_cd)      
    rospy.Subscriber('/msg_et',String,AchieveGoal_et)
    rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,AmclStartGoal)
    rospy.Subscriber('/number_obstacles',String,sub_number_obstacles)
    rospy.Subscriber('/risk_obstacles',String,sub_risk_obstacles)

    pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    pub_initialpose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
    pub_start_calculate = rospy.Publisher('/start_calculate', String, queue_size=1)
    pub_start_switch = rospy.Publisher('/start_switch', String, queue_size=1)##  用于开始 excution_time 的计算
    
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    for j in cylinder_number:
        for i in range(world_number_start,world_number_end):
            all_combine_average_value = {
                'global_planner/GlobalPlanner + eband_local_planner/EBandPlannerROS':{
                    'sum':0,'success_count':0,'fail_count':0
                },
                'global_planner/GlobalPlanner + dwa_local_planner/DWAPlannerROS':{
                    'sum':0,'success_count':0,'fail_count':0
                },
                'global_planner/GlobalPlanner + base_local_planner/TrajectoryPlannerROS':{
                    'sum':0,'success_count':0,'fail_count':0
                },
                'navfn/NavfnROS + eband_local_planner/EBandPlannerROS':{
                    'sum':0,'success_count':0,'fail_count':0
                },
                'navfn/NavfnROS + dwa_local_planner/DWAPlannerROS':{
                    'sum':0,'success_count':0,'fail_count':0
                },
                'navfn/NavfnROS + base_local_planner/TrajectoryPlannerROS':{
                    'sum':0,'success_count':0,'fail_count':0
                }}
            save_dir = '/home/zlb/dev/ros_collections/catkin_ws_adaptive_switch_algorithm/src/adaptive_algorithm/maps/slim_cylinder_' + str(j) + '/world_cylinder_'
            
            # 设置环境变量
            os.environ['MAP_FILE'] = save_dir + str(j) + '_' + str(i) + '.yaml'
            os.environ['WORLD_DIR'] = save_dir + str(j) + '_' + str(i) + '.world'
            time.sleep(1)

            # 启动gazebo
            gazebo_launch = roslaunch.parent.ROSLaunchParent(uuid, [turtlebot3_world_launch])
            gazebo_launch.start()
            time.sleep(15)
        
            # 启动导航rviz
            nav_launch = roslaunch.parent.ROSLaunchParent(uuid, [turtlebot3_navigation_launch])
            nav_launch.start()
            time.sleep(10)
        
            # 发布初始位置信息
            try:
                msg_initialpose.header.stamp = rospy.Time.now()
                msg_initialpose.header.frame_id = "map"
                pub_initialpose.publish(msg_initialpose)
            except KeyError as e:
                print(e)
            time.sleep(5)
            rospy.loginfo(" # 发布初始位置信息")
            pub_start_calculate.publish("1")
            
            
            time.sleep(10)
            for bgp in bgp_all: # 切换不同全局算法
                base_global_planner = bgp 
                rospy.loginfo(" # 切换不同全局算法")
                for blp in blp_all: # 切换不同局部算法
                    rospy.loginfo(" # 切换不同局部算法")
                    base_local_planner = blp
                    # 动态调参 调整全局和局部参数
                    # msg_algorithmn_combine = "rosrun dynamic_reconfigure dynparam set /move_base '{'base_global_planner': "\
                    # + base_global_planner +", 'conservative_reset_dist': 3.0, 'groups': {'base_global_planner': "\
                    # + base_global_planner + ", 'planner_frequency': 5.0, 'parent': 0, 'conservative_reset_dist': 3.0, 'shutdown_costmaps': False, 'restore_defaults': False, 'groups': {}, 'oscillation_timeout': 10.0, 'id': 0, 'controller_patience': 15.0, 'name': 'Default', 'parameters': {}, 'type': '', 'clearing_rotation_allowed': True, 'state': True, 'oscillation_distance': 0.2, 'max_planning_retries': -1, 'base_local_planner': " \
                    # + base_local_planner + " , 'recovery_behavior_enabled': True, 'planner_patience': 5.0, 'controller_frequency': 10.0}, 'controller_patience': 15.0, 'max_planning_retries': -1, 'shutdown_costmaps': False, 'clearing_rotation_allowed': True, 'restore_defaults': False, 'oscillation_distance': 0.2, 'planner_frequency': 5.0, 'oscillation_timeout': 10.0, 'base_local_planner': " \
                    # + base_local_planner + "',recovery_behavior_enabled': True, 'planner_patience': 5.0, 'controller_frequency': 10.0}'"
                    msg_algorithmn_combine = "rosrun dynamic_reconfigure dynparam set /move_base '{'base_global_planner': "\
                    + base_global_planner +", 'groups': {'base_global_planner': "\
                    + base_global_planner + ", 'base_local_planner': " \
                    + base_local_planner + " }, 'base_local_planner': " \
                    + base_local_planner + "}'"
                    os.system(msg_algorithmn_combine)
                    time.sleep(2.3)
                    algorithmn_combine = base_global_planner + " + " + base_local_planner
                    rospy.loginfo("-----------bgp/blp-------" + bgp + " + " + blp)
                    rospy.loginfo("-----------algorithmn_combine:" + str(algorithmn_combine))
                    rospy.loginfo("-----------param:" + rospy.get_param("/move_base/base_global_planner") + " + " + rospy.get_param("/move_base/base_local_planner"))
                    
                    pub_start_switch.publish("开始新地图，准备计算执行时间")

                    # for k in range(10):# 单种组合
                    # 发布目标位置信息
                    try:
                        msg_goal.header.stamp = rospy.Time.now()
                        msg_goal.header.frame_id = "map"
                        pub_goal.publish(msg_goal) 
                    except KeyError as e:
                        print(e)
                        msg_goal.header.stamp = rospy.Time.now()
                        msg_goal.header.frame_id = "map"
                        pub_goal.publish(msg_goal) 
                    rospy.loginfo(" # 发布目标位置信息")
                    # 此处会监听是否到达终点,如果到达就进行下一步,如果60s还没到达,默认路径规划失败
                    count = 0
                    switch_count = 0
                    while  not achieve_goal_flag and (count<600):
                        time.sleep(0.1)
                        if base_local_planner != rospy.get_param("/move_base/base_local_planner"):
                            switch_count += 1
                            base_local_planner = rospy.get_param("/move_base/base_local_planner")
                            try:
                                msg_goal.header.stamp = rospy.Time.now()
                                msg_goal.header.frame_id = "map"
                                pub_goal.publish(msg_goal) 
                            except KeyError as e:
                                print(e)
                                msg_goal.header.stamp = rospy.Time.now()
                                msg_goal.header.frame_id = "map"
                                pub_goal.publish(msg_goal) 
                        count += 1
                    time.sleep(1)
                    got_pv_flag = True

                    # 取消目标
                    try:
                        msg_cancel.stamp = rospy.Time.now()
                        pub_cancel.publish(msg_cancel)
                    except KeyError as e:
                        print(e)
                    # os.system("rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}")
                    time.sleep(10)
                    cost_time = count*1.0/100
                    rospy.loginfo("花费了约" + str(cost_time) + "秒")
                    
                    # 到达目标点,获得pv,比较之后写入 pv_best,如果是超时跳出循环的,那么它的pv就不进行比较了
                    if count == 599:
                        rospy.loginfo("路径规划失败")
                    elif plan_evaluation > plan_evaluation_max:
                        plan_evaluation_max = plan_evaluation
                        algorithmn_combine_best = algorithmn_combine
                        pv_best['algorithmn_combine'] = algorithmn_combine_best
                        pv_best['plan_evaluation'] = plan_evaluation_max

                    
                    feature = str(number_obstacles) + " " + str(distance_each_other_obstacles_average) + " " + str(ocupancy_obstacles) + " " + str(risk_obstacles)
                    with open(file_name + '_every_combine', 'a+') as f:
                        f.write('slim_cylinder_' + str(j) + '/world_cylinder_'+str(j)+'_' + str(i) + ":\n" + 
                        "feature:" + str(feature) + "\n" + 
                        "algorithmn_combine: " + str(algorithmn_combine) + "\nplan_evaluation: " + 
                        str(plan_evaluation)  + "\nexcution_time: " + 
                        str(et)   + "\ncost_time: " + 
                        str(cost_time) + "\nswitch_count: " + 
                        str(switch_count)+ "\n ------------ \n")
                # 调用 rosservice 将小车恢复到原来的位置
                    # # 现在返回起始点  gazebo中
                    os.system('rosservice call /gazebo/reset_world')
                    # 发布初始位置信息  rviz中
                    try:
                        msg_initialpose.header.stamp = rospy.Time.now()
                        msg_initialpose.header.frame_id = "map"
                        pub_initialpose.publish(msg_initialpose)
                    except KeyError as e:
                        print(e)
                    time.sleep(1)
                    
                    # # 现在返回起始点  gazebo中 ,使用两次是为了能够确保回到
                    os.system('rosservice call /gazebo/reset_world')
                    # 发布初始位置信息  rviz中
                    try:
                        msg_initialpose.header.stamp = rospy.Time.now()
                        msg_initialpose.header.frame_id = "map"
                        pub_initialpose.publish(msg_initialpose)
                    except KeyError as e:
                        print(e)
                    time.sleep(1)
                    
                    # 清除代价地图,以免对后一个算法造成影响
                    os.system('rosservice call /move_base/clear_costmaps')
                    '''
                    # ######################写入 trajectory_length , closest_distance########################
                    # if trajectory_length == 0:
                    #     all_combine_average_value[algorithmn_combine]['fail_count'] += 1
                    #     # distance_each_other_obstacles open(file_name, 'a+') as f:
                    #     #     f.write( "world_cylinder_" + str(j) +'_'+ str(i) + ":\n" + \
                    #     #     "algorithmn_combine: " + algorithmn_combine + \
                    #     #     "\n 规划失败！！！！！！" + ":\n" + \
                    #     #     str(all_combine_average_value[algorithmn_combine]) + "\n "\
                    #     #     "********* \n\n")

                    # else:
                    #     all_combine_average_value[algorithmn_combine]['sum'] += 100*closest_distance/trajectory_length
                    #     all_combine_average_value[algorithmn_combine]['success_count'] += 1
                    #     # distance_each_other_obstacles open(file_name, 'a+') as f:
                    #     #     f.write( "world_cylinder_" + str(j) +'_'+ str(i) + ":\n" + \
                    #     #     "algorithmn_combine: " + algorithmn_combine + "\n"+ \
                    #     #     "trajectory_length: " + str(trajectory_length) + "\n"+ \
                    #     #     "closest_distance: " + str(closest_distance) + "\n" + \
                    #     #     "closest_distance/trajectory_length: " + str(100*closest_distance/trajectory_length) + "\n"+ \
                    #     #     str(all_combine_average_value[algorithmn_combine]) + "\n" + \
                    #     #     "********* \n\n")
                    '''
                    # 值设为初始值
                    achieve_start_flag = True
                    achieve_goal_flag = False
                    got_pv_flag = False
                    plan_evaluation = 0
                    trajectory_length = 0
                    closest_distance = 0
                    '''
                    # sum_value = all_combine_average_value[algorithmn_combine]['sum']
                    # success_count = all_combine_average_value[algorithmn_combine]['success_count']
                    # distance_each_other_obstacles open(file_name, 'a+') as f:
                    #         f.write( "world_cylinder_" + str(j) +'_'+ str(i) + ":\n" + \
                    #         "algorithmn_combine: " + algorithmn_combine + "\n"+ \
                    #         "sum: " + str(sum_value) + "\n"+ \
                    #         "success_count: " + str(success_count) + "\n"+ \
                    #         "average: " + str(1.0*sum_value/success_count) + "\n"+ \
                    #         "fail_count: " + str(all_combine_average_value[algorithmn_combine]['fail_count']) + "\n"+ \
                    #         "********* \n\n" + "----------------------------------------------------------------------------\n\n")
                
              
                    '''              
            with open(file_name + '_every_combine', 'a+') as f:
                f.write( "\n ------------ \n")
                ###################### 写入 pv  勿删 ########################
            feature = str(number_obstacles) + " " + str(distance_each_other_obstacles_average) + " " + str(ocupancy_obstacles) + " " + str(risk_obstacles)
            with open(file_name, 'a+') as f:
                f.write('slim_cylinder_' + str(j) + '/world_cylinder_'+str(j)+'_' + str(i) + ":\n" + 
                "feature:" + str(feature) + "\n" + 
                "algorithmn_combine_best: " + str(pv_best['algorithmn_combine']) + "\nplan_evaluation: " + 
                str(pv_best['plan_evaluation']) + "\n ------------ \n")
            
            #换新地图,最佳算法组合清零,最佳评价函数清零
            algorithmn_combine_best = ""
            plan_evaluation_max = 0        
            nav_launch.shutdown()
            gazebo_launch.shutdown()
            number_obstacles = 0
            distance_each_other_obstacles_average = 0
            ocupancy_obstacles = 0
            risk_obstacles = 0
            time.sleep(10)

    rospy.spin()
def sub_number_obstacles(msg):
    global number_obstacles,distance_each_other_obstacles_average,ocupancy_obstacles

    rospy.loginfo("set number_obstacles,distance_each_other_obstacles_average,ocupancy_obstacles")
    data_list = msg.data.split(" - ")
    rospy.loginfo(msg)
    ## 获得障碍物数量
    number_obstacles = int(data_list[0])
    center_x_y_r_list = eval(data_list[1])
    ocupancy_obstacles = eval(data_list[2])
    
    # rospy.loginfo(str(center_x_y_r_list) + "-----\n")
    # 计算各个障碍物之间的距离,接收到的数据里有center_x_y_r,已知坐标以及半径的两个圆之间计算距离.
    distance_each_other_obstacles_sum = 0
    distance_each_other_obstacles = 0
    distance_each_other_list = []   
    for i in range(len(center_x_y_r_list)):
        length = i + 1
        while length < len(center_x_y_r_list):
            distance_each_other_single = []
            distance_each_other_single.append(i)
            distance_each_other_single.append(length)
            distance_each_other_obstacles=int(abs(
                math.sqrt((center_x_y_r_list[length][0]-center_x_y_r_list[i][0])**2+(center_x_y_r_list[length][1]-center_x_y_r_list[i][1])**2)
                -center_x_y_r_list[length][2]-center_x_y_r_list[i][2]))
            distance_each_other_list.append(distance_each_other_single)
            distance_each_other_obstacles_sum += distance_each_other_obstacles
            length += 1
    rospy.loginfo("--------distance_each_other_obstacles_average--------" + str(distance_each_other_obstacles_average))
    distance_each_other_obstacles_average = distance_each_other_obstacles_sum*1.0/len(distance_each_other_list)
            
    rospy.loginfo(str(distance_each_other_list) + "-----\n")

def sub_risk_obstacles(msg):
    global risk_obstacles

    risk_obstacles = eval(msg.data)
    rospy.loginfo("set risk_obstacles" + str(risk_obstacles))

def AchieveGoal_cd(msg):
    # 到达终点才记录pv
    global achieve_goal_flag,got_pv_flag,closest_distance
    if achieve_goal_flag and not got_pv_flag:
        rospy.loginfo("到达目标点,获取pv")
        closest_distance = float(msg.data.split(":")[1])

def AchieveGoal_tl(msg):
    # 到达终点才记录pv
    global achieve_goal_flag,got_pv_flag,trajectory_length
    if achieve_goal_flag and not got_pv_flag:
        rospy.loginfo("到达目标点,获取pv")
        trajectory_length = float(msg.data.split(":")[1])

def AchieveGoal_pv(msg):
    # 到达终点才记录pv
    global pv_best,algorithmn_combine,algorithmn_combine_best,plan_evaluation,plan_evaluation_max
    global achieve_start_flag,achieve_goal_flag,got_pv_flag
    if achieve_goal_flag and not got_pv_flag:
        rospy.loginfo("到达目标点,获取pv")
        plan_evaluation = float(msg.data.split(":")[1])


def AmclStartGoal(msg):
    '''监听 /amcl 看自己是回到起始点,,最后位置与起始位置小于阈值时算回到起始点,这时候 achieve_start_flag 改为 True'''
    #订阅到的小车的坐标信息
    global achieve_start_flag,achieve_goal_flag
    global start_x,start_y,goal_x,goal_y

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y


    ## 如果achieve_start_flag ,那么 

    if (math.sqrt((x-goal_x)**2+(y-goal_y)**2) < 0.5):
        rospy.loginfo("到达目标点,设置flag")
        achieve_goal_flag = True
        achieve_start_flag = False

    # if (math.sqrt((x-start_x)**2+(y-start_y)**2) < 0.5):
    #     rospy.loginfo("到达起始点,设置flag")
    #     achieve_goal_flag = False
    #     achieve_start_flag = True

def AchieveGoal_et(msg):
    # 到达终点才记录pv
    global et
    if achieve_goal_flag and not got_pv_flag:
        rospy.loginfo("到达目标点,获取et")
        et = float(msg.data.split(":")[1])
if __name__ == '__main__':
    main()