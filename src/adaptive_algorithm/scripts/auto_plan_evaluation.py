#!/usr/bin/python3
# -*- coding: utf-8 -*-
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
msg_initialpose.pose.pose.position.x = 2
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
msg_goal.pose.position.x = 8
msg_goal.pose.position.y = 8
msg_goal.pose.position.z = 0
msg_goal.pose.orientation.x = 0
msg_goal.pose.orientation.y = 0
msg_goal.pose.orientation.z = 0
msg_goal.pose.orientation.w = 1

# 发布起始点位置
# header部分
msg_start = PoseStamped()
# pose部分,包括 坐标position 和 方向orientation
msg_start.pose.position.x = 2
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

start_x = 2 # 起始点的坐标
start_y = 1
goal_x = 8 # 目标点的坐标
goal_y = 8
world_number_start = 300 # 第几张图开始
world_number_end = 400 # 第几张图结束

cylinder_number = [10] # 有的障碍物数量
bgp_all = ["navfn/NavfnROS","global_planner/GlobalPlanner"] # 全局规划算法的集合
blp_all = ["eband_local_planner/EBandPlannerROS","dwa_local_planner/DWAPlannerROS","base_local_planner/TrajectoryPlannerROS"] # 局部....

pv_best = {"algorithmn_combine":"base","plan_evaluation":0} # 字典形式的
file_name = '/home/zlb/auto_plan_evaluation.txt'
algorithmn_combine = "" # 单次规划中的算法组合 初始化空字符串
algorithmn_combine_best = "" # 单个环境中的最优算法组合 初始化空字符串
plan_evaluation = 0 # 评价函数
plan_evaluation_max = 0 # 最高评价函数

global_planner = "navfn/NavfnROS" # 默认为 "navfn/NavfnROS"
local_planner = "base_local_planner/TrajectoryPlannerROS" # 默认为 "base_local_planner/TrajectoryPlannerROS"

# 动态调参 调整全局和局部参数
msg_algorithmn_combine = "rosrun dynamic_reconfigure dynparam set /move_base '{'base_global_planner': "\
 + global_planner +", 'conservative_reset_dist': 3.0, 'groups': {'base_global_planner': "\
 + global_planner + ", 'planner_frequency': 5.0, 'parent': 0, 'conservative_reset_dist': 3.0, 'shutdown_costmaps': False, 'restore_defaults': False, 'groups': {}, 'oscillation_timeout': 10.0, 'id': 0, 'controller_patience': 15.0, 'name': 'Default', 'parameters': {}, 'type': '', 'clearing_rotation_allowed': True, 'state': True, 'oscillation_distance': 0.2, 'max_planning_retries': -1, 'base_local_planner': " \
 + local_planner + " , 'recovery_behavior_enabled': True, 'planner_patience': 5.0, 'controller_frequency': 10.0}, 'controller_patience': 15.0, 'max_planning_retries': -1, 'shutdown_costmaps': False, 'clearing_rotation_allowed': True, 'restore_defaults': False, 'oscillation_distance': 0.2, 'planner_frequency': 5.0, 'oscillation_timeout': 10.0, 'base_local_planner': " \
 + local_planner + "',recovery_behavior_enabled': True, 'planner_patience': 5.0, 'controller_frequency': 10.0}'"


def main():
    global msg_algorithmn_combine,msg_gazebo,msg_goal,msg_initialpose,msg_rviz,msg_start
    global achieve_start_flag,achieve_goal_flag,got_pv_flag
    global start_x,start_y,goal_x,goal_y
    global bgp_all,blp_all,world_number,cylinder_number
    global pv_best,algorithmn_combine,algorithmn_combine_best,plan_evaluation,plan_evaluation_max
    global global_planner,local_planner
    global msg_initialpose,msg_goal
    
    rospy.init_node('auto_plan_evaluation',anonymous=False)        
    rospy.Subscriber('/msg_pv',String,AchieveGoal)
    rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,AmclStartGoal)
    pub_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    pub_initialpose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    pub_cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    turtlebot3_world_launch = "/home/zlb/dev/ros_collections/catkin_ws_adaptive_switch_algorithm/src/adaptive_algorithm/launch/turtlebot3_world.launch"
    turtlebot3_navigation_launch = "/home/zlb/dev/ros_collections/catkin_ws_adaptive_switch_algorithm/src/adaptive_algorithm/launch/turtlebot3_navigation.launch"

    for j in cylinder_number:
        for i in range(world_number_start,world_number_end):
            for k in range(10):
                # 设置环境变量
                os.environ['MAP_FILE'] = '/home/zlb/dev/ros_collections/catkin_ws_adaptive_switch_algorithm/src/adaptive_algorithm/maps/slim_cylinder_10/world_cylinder_' + str(j) + '_' + str(i) + '.yaml'
                os.environ['WORLD_DIR'] = '/home/zlb/dev/ros_collections/catkin_ws_adaptive_switch_algorithm/src/adaptive_algorithm/maps/slim_cylinder_10/world_cylinder_' + str(j) + '_' + str(i) + '.world'
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
                
                for bgp in bgp_all: # 切换不同全局算法
                    global_planner = bgp 
                    for blp in blp_all: # 切换不同局部算法
                        local_planner = blp
                        # 动态调参 调整全局和局部参数
                        msg_algorithmn_combine = "rosrun dynamic_reconfigure dynparam set /move_base '{'base_global_planner': "\
                        + global_planner +", 'conservative_reset_dist': 3.0, 'groups': {'base_global_planner': "\
                        + global_planner + ", 'planner_frequency': 5.0, 'parent': 0, 'conservative_reset_dist': 3.0, 'shutdown_costmaps': False, 'restore_defaults': False, 'groups': {}, 'oscillation_timeout': 10.0, 'id': 0, 'controller_patience': 15.0, 'name': 'Default', 'parameters': {}, 'type': '', 'clearing_rotation_allowed': True, 'state': True, 'oscillation_distance': 0.2, 'max_planning_retries': -1, 'base_local_planner': " \
                        + local_planner + " , 'recovery_behavior_enabled': True, 'planner_patience': 5.0, 'controller_frequency': 10.0}, 'controller_patience': 15.0, 'max_planning_retries': -1, 'shutdown_costmaps': False, 'clearing_rotation_allowed': True, 'restore_defaults': False, 'oscillation_distance': 0.2, 'planner_frequency': 5.0, 'oscillation_timeout': 10.0, 'base_local_planner': " \
                        + local_planner + "',recovery_behavior_enabled': True, 'planner_patience': 5.0, 'controller_frequency': 10.0}'"
                        os.system(msg_algorithmn_combine)
                        time.sleep(2)
                        algorithmn_combine = global_planner + " + " + local_planner
                        rospy.loginfo("-----------bgp/blp-------" + bgp + " + " + blp)
                        rospy.loginfo("-----------algorithmn_combine:" + str(algorithmn_combine))
                        rospy.loginfo("-----------param:" + rospy.get_param("/move_base/base_global_planner") + " + " + rospy.get_param("/move_base/base_local_planner"))
                        # 发布目标位置信息
                        try:
                            msg_goal.header.stamp = rospy.Time.now()
                            msg_goal.header.frame_id = "map"
                            pub_goal.publish(msg_goal) 
                        except KeyError as e:
                            print(e)
                        
                        # 此处会监听是否到达终点,如果到达就进行下一步,如果60s还没到达,默认路径规划失败
                        count = 0
                        while  not achieve_goal_flag and (count<600):
                            time.sleep(0.1)
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
                        # time.sleep(5)
                        rospy.loginfo("花费了约" + str(count/10) + "秒")
                        # 到达目标点,获得pv,比较之后写入 pv_best,如果是超时跳出循环的,那么它的pv就不进行比较了
                        if count == 599:
                            rospy.loginfo("路径规划失败")
                        elif plan_evaluation > plan_evaluation_max:
                            plan_evaluation_max = plan_evaluation
                            algorithmn_combine_best = algorithmn_combine
                            pv_best['algorithmn_combine'] = algorithmn_combine_best
                            pv_best['plan_evaluation'] = plan_evaluation_max

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
                        
                        # 将
                        achieve_start_flag = True
                        achieve_goal_flag = False
                        got_pv_flag = False
                        plan_evaluation = 0

                with open(file_name, 'a+') as f:
                    f.write('/home/zlb/dev/ros_collections/catkin_ws_adaptive_switch_algorithm/src/adaptive_algorithm/maps/slim_cylinder_10/world_cylinder_10_' + str(i) + ":\n" + "algorithmn_combine_best: " + str(pv_best['algorithmn_combine']) + "\nplan_evaluation: " + str(pv_best['plan_evaluation']) + "\n ------------ \n")
                # 换新地图,最佳算法组合清零,最佳评价函数清零
                algorithmn_combine_best = ""
                plan_evaluation_max = 0        
                nav_launch.shutdown()
                gazebo_launch.shutdown()
                time.sleep(10)
    rospy.spin()

def AchieveGoal(msg):
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


if __name__ == '__main__':
    main()
