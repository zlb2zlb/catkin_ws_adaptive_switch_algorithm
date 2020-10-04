#!/bin/sh
roscore & python tl1.py & python test_kkk.py & python count_obstacles.py & python start_navigation.py
# 启动tl1.py,以提供评价函数pv
# python tl1.py
# ##启动test_kkk.py,以提供威胁程度计算
# python test_kkk.py
# ##启动count_obstacles.py,以提供障碍物信息计算
# python count_obstacles.py
# ## 启动导航
# python start_navigation.py
