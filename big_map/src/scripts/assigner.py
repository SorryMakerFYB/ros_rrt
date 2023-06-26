#!/usr/bin/env python3

#--------Include modules---------------
from std_msgs.msg import Int32
from copy import copy
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,Twist
from nav_msgs.msg import OccupancyGrid,Odometry
import tf
from big_map.msg import PointArray
from time import time
from numpy import array
from numpy import linalg as LA
from numpy import all as All
from numpy import inf
from numpy.linalg import norm
import numpy as np
def point_of_index(mapData, i):
    y = mapData.info.origin.position.y + \
        (i//mapData.info.width)*mapData.info.resolution
    x = mapData.info.origin.position.x + \
        (i-(i//mapData.info.width)*(mapData.info.width))*mapData.info.resolution
    return array([x, y])
class assigner():
    meeting_start=rospy.Time.now()
    meeting_end=rospy.Time.now()
    def __init__(self):
        self.subodom1map=rospy.Subscriber("/map_merge/map",OccupancyGrid,self.mapCallback)
        self.subodom1=rospy.Subscriber("/robot_1/odom",Odometry,self.odom1)
        self.subodom2=rospy.Subscriber("/robot_2/odom",Odometry,self.odom2)
        self.subodom3=rospy.Subscriber("/robot_3/odom",Odometry,self.odom3)
        self.map=OccupancyGrid()
        self.odom1=Odometry()
        self.odom2=Odometry()
        self.odom3=Odometry()
        self.pubCentroidsMeetPoint=rospy.Publisher("/centroids",PointArray,queue_size=10)
        self.pubflag=rospy.Publisher("/stage_flag",Int32,queue_size=10)
        self.assign_flag=0 #开会中，分配中心点和下一次开会地点和开会时间
        self.pub_vel=rospy.Publisher("cmd_vel",Twist,queue_size=10)
        #一直往前走的速度
        self.vel_forward=Twist()
        self.vel_forward.linear.x=1
        self.vel_forward.linear.y=0
        self.vel_forward.linear.z=0
        self.vel_forward.angular.z=0

    def odom1(self,odom):
        self.odom1=odom
        
    def odom2(self,odom):
        self.odom=odom
    
    def odom3(self,odom):
        self.odom=odom
        
    def mapCallback(self,map):
        self.pub_vel(self.vel_forward)
        self.map=map
        free=array([])
        for i in map.data:
            if i==-1:
                np.vstack(free,copy(point_of_index(self.map,i.index())))
        #如果哦flag:0 开会中
        #flag:1 探索中
        #flag:2 回去开会中
        if self.assign_flag==0:
            for i in map.data:
             if i==-1:
                np.vstack(free,copy(point_of_index(self.map,i.index())))
            rospy.loginfo(free)
            #这里把地图kmeans得到3个点和一个开会地点加进去发布
            centroidsAndmeetpoint=[]
            self.pubCentroidsMeetPoint(centroidsAndmeetpoint)
            self.assign_flag=1
            assigner.meeting_start=rospy.Time.now()
            
        elif self.assign_flag==1:
            #判断是否达到开会时间，如果达到了，就去,记时30s，时间到把flag=2
            #if time_to_meet() 
            assigner.meeting_end=rospy.Time.now()
            if(assigner.meeting_end.secs-assigner.meeting_start.secs>30):
                self.assign_flag=2
        elif self.assign_flag==2:
            #取消发送所有目标点 ，send_goal(meeeting_points)
            pass
            
        
        self.pubflag(self.assign_flag)

        


        
if __name__ == '__main__':
    rospy.init_node("assigner")
    a=assigner()
 
 
 
 
