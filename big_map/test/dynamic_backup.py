#!/usr/bin/env python3 
# coding=utf-8
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry,OccupancyGrid
from numpy import floor
from sklearn.cluster import MeanShift ,KMeans
from numpy import array
import actionlib
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped,PoseWithCovariance,Point
from big_map.msg import PointArray
from numpy.linalg import norm
from copy import copy
# 注意在Python的类中，是通过self来表示某个变量是其成员的
# 如果不通过self方式获取，Python就会认为这个变量是全局变量
class Processer:
    goal = MoveBaseGoal()
    
    def __init__(self,robot_name):
        self.map=OccupancyGrid()
        self.map.header.frame_id="map"
        self.map.header.stamp=rospy.Time.now()
        self.map.info.width=100
        self.map.info.height=100
        #测试起始位置在那里是正中间
        self.map.info.origin.position.x=-100
        self.map.info.origin.position.y=-100
        self.map.info.resolution=2.0
        self.subscribe_name=robot_name+"/odom"
        self.map.data=[-1]*self.map.info.width*self.map.info.height
        self.sub1 = rospy.Subscriber("odom", Odometry, self.callback1)      
        self.pub1 = rospy.Publisher("map", OccupancyGrid, queue_size=1)
        self.sub2 =rospy.Subscriber("detected_points",PointArray,self.choose_best)
        self.free=set()#已探索
        self.assigned_centroids=Point()
        self.assigned_centroids.x=0
        self.assigned_centroids.y=0
        self.assigned_centroids.z=0
        self.pose=PoseWithCovariance()
        #初始化action客户端
        self.client = actionlib.SimpleActionClient(
            robot_name, MoveBaseAction)
        self.client.wait_for_server()
    def choose_best(self,p):
        #从边缘点选一个出来：
        cost=[]
        def angle(a,b):
            cos_ = np.dot(a,b)/(norm(a)*norm(b))
            # 夹角sin值
            sin_ = np.cross(a,b)/(norm(a)*norm(b))
            arctan2_ = np.arctan2(sin_, cos_)
            
            return arctan2_/np.pi
        for i in p.points:
            a=np.array([self.pose.pose.position.x,self.pose.pose.position.y])
            b=np.array([i.x-self.pose.pose.position.x,i.y-self.pose.pose.position.y])
            angle_between=angle(a,b)
            c=np.array([i.x-self.assigned_centroids.x,i.y-self.assigned_centroids.y])
            value=(angle_between+1)*norm(b)+norm(c)#防止乘0
            cost.append(copy(value))
        best=cost.index(max(cost))
        #rospy.loginfo(p.points[best])
        return(p.points[best])
    
    
    def callback1(self, odom):
        self.pose=odom.pose
        point=[odom.pose.pose.position.x,odom.pose.pose.position.y]
        indices=self.index_of_point(point)
        for index in indices:
            self.free.add(index)
            self.map.data[index]=0
        self.pub1.publish(self.map)

    def index_of_point(self, Xp):
        resolution = self.map.info.resolution
        Xstartx = self.map.info.origin.position.x
        Xstarty = self.map.info.origin.position.y
        width = self.map.info.width
        Data = self.map.data
        indices=[]
        for i in range(-10,10):
            for j in range(-10,10):
                index = int(	(floor((Xp[1]+j-Xstarty)/resolution) *
                    width)+(floor((Xp[0]+i-Xstartx)/resolution)))
                indices.append(index)
        return indices
    




if __name__ == "__main__":
    rospy.init_node("subscribe_publish")
    ns=rospy.get_param("~robot_name",'')
    p = Processer(ns)

    rospy.spin()