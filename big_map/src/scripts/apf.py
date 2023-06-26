#!/usr/bin/env python3 
# coding=utf-8
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32,Int32
from nav_msgs.msg import Odometry,OccupancyGrid
from numpy import floor
from sklearn.cluster import MeanShift ,KMeans
from numpy import array
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped,PoseWithCovariance,Point,Twist
from big_map.msg import PointArray
from numpy.linalg import norm
from copy import copy
#一个apf路径规划其，传入位置和目标和地图。
def apf_planner(sx,sy,gx,gy,map):
    KP = 5.0  # attractive potential gain
    ETA = 100.0  # repulsive potential gain

    def calc_attractive_potential(sx, sy, gx, gy):
        return 0.5 * KP * np.hypot(sx - gx, sy - gy)
    def calc_repulsive_potential(x, y, ox, oy, rr):
    # search nearest obstacle
    #传入ox,oy这个是障碍物数组
        minid = -1
        dmin = float("inf")
        for i, _ in enumerate(ox):
            d = np.hypot(x - ox[i], y - oy[i])
            if dmin >= d:
                dmin = d
                minid = i

        # calc repulsive potential
        dq = np.hypot(x - ox[minid], y - oy[minid])

        if dq <= rr:
            if dq <= 0.1:
                dq = 0.1

            return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
        else:
            return 0.0