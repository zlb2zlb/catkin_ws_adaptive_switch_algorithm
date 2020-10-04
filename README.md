# catkin_ws_adaptive_switch_algorithm
自动选择算法

# 启动
python src/adaptive_algorithm/scripts/count_obstacles.py
# 启动roscore
roscore
# 启动tl1.py,以提供评价函数pv
python src/adaptive_algorithm/scripts/tl1.py
# 启动test_kkk.py,以提供威胁程度计算
python src/adaptive_algorithm/scripts/test_kkk.py
# 启动count_obstacles.py,以提供障碍物信息计算
python src/adaptive_algorithm/scripts/count_obstacles.py
# 启动test_auto_plan_evaluation.py,以开启gazebo rviz开始跑程序
# python src/adaptive_algorithm/scripts/test_auto_plan_evaluation.py
python src/adaptive_algorithm/scripts/start_navigation.py