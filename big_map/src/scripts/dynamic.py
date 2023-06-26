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
from actionlib_msgs.msg import GoalID
import tf
# 注意在Python的类中，是通过self来表示某个变量是其成员的
# 如果不通过self方式获取，Python就会认为这个变量是全局变量
class Processer:
   
    
    def __init__(self,robot_name):
        self.map=OccupancyGrid()
        self.map.header.frame_id="map"
        self.map.header.stamp=rospy.Time.now()
        self.map.info.width=100
        self.map.info.height=100
        #测试起始位置在那里是正中间
        self.map.info.origin.position.x=-50
        self.map.info.origin.position.y=-50
        self.map.info.resolution=1
        self.subscribe_name=robot_name+"/odom"
        self.map.data=[-1]*self.map.info.width*self.map.info.height
        #目标点构造
        self.Best_goal = MoveBaseGoal()
        self.Best_goal.target_pose.header.frame_id = "map"
        self.Best_goal.target_pose.header.stamp = rospy.Time.now()
         
        #一直往前走的速度
        self.pub_vel=rospy.Publisher("cmd_vel",Twist,queue_size=10)
        self.vel_forward=Twist()
        self.vel_forward.linear.x=0.1
        self.vel_forward.linear.y=0
        self.vel_forward.linear.z=0
        self.vel_forward.angular.z=0

        
        #接受消息，和回调函数
        self.sub1 = rospy.Subscriber("odom", Odometry, self.callback1)      
        self.pub1 = rospy.Publisher("map", OccupancyGrid, queue_size=1)
        self.sub2 =rospy.Subscriber("detected_points",PointArray,self.choose_best)
        self.free=set()#已探索
        self.assigned_centroids=Point()
        self.assigned_centroids.x=0
        self.assigned_centroids.y=0
        self.assigned_centroids.z=0
        self.pose=PoseWithCovariance()

        #往前走的点
        self.forward_point=Point()
        self.forward_point.x=self.pose.pose.position.x+15
        self.forward_point.y=self.pose.pose.position.y
        #初始化action客户端
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

        
        self.sub3=rospy.Subscriber("/stage_flag",Int32,self.flag_callback)
        self.stage_flag=0
        #接受探索阶段标志位：0开会中1探索中2回去开会
        #0阶段原地盘旋，1阶段自由探索，2阶段goal是开会地点

       
    def flag_callback(self,flag):
        self.stage_flag=flag
        if self.stage_flag==0:
            #这里应该是取消所有目标开始盘旋
            pass
        elif self.stage_flag==1:
            #开始探索
            pass
        elif self.stage_flag==2:
            #取消所有目标，设置目标为开会地点
            pass
    def explore(self,p):
        pass
    def hover(self):
        cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        cancel_msg = GoalID()
        cancel_pub.publish(cancel_msg)
    def choose_best(self,p):
            #从边缘点选一个出来：flag=1的时候
            #self.pub_vel.publish(self.vel_forward) #不能一直给一个向前的速度
            cost=[]
            def angle(a,b):
                #计算角度为后面costfunction提供角度项
                cos_ = np.dot(a,b)/(norm(a)*norm(b))
                # 夹角sin值
                sin_ = np.cross(a,b)/(norm(a)*norm(b))
                arctan2_ = np.arctan2(sin_, cos_)
                return arctan2_
            for i in p.points:
                a=np.array([self.pose.pose.position.x,self.pose.pose.position.y])
                b=np.array([i.x-self.pose.pose.position.x,i.y-self.pose.pose.position.y])
                angle_between=angle(a,b)
                c=np.array([i.x-self.assigned_centroids.x,i.y-self.assigned_centroids.y])
                value=(angle_between+1)*norm(b)#+norm(c)#防止乘0
                cost.append(copy(value))
            best=cost.index(max(cost))
            if self.getState()==0:#未被处理
                self.sendGoal(p.points[best])
            elif self.getState()==1:#在处理中，不操作
                pass
            elif self.getState()==2:#被取消了，而且已经取消完了
                self.sendGoal(p.points[best])
            elif self.getState()==3:#实现了，到达了     
                self.sendGoal(p.points[best])
            elif self.getState()==4:#到达不了了，多次失败，先往前走点
                self.client.cancel_all_goals()
                
                self.sendGoal(self.forward_point)
            elif self.getState()==5:
                pass
            elif self.getState()==6:
                pass
            elif self.getState()==7:
                pass
            elif self.getState()==8:
                pass
            elif self.getState()==9:
                self.sendGoal(p.points[best])
            rospy.loginfo(self.getState())
            return(p.points[best])
    def sendGoal(self, point):
        self.Best_goal.target_pose.header.frame_id = "map"
        self.Best_goal.target_pose.header.stamp = rospy.Time.now()
        self.Best_goal.target_pose.pose.position.x = point.x
        self.Best_goal.target_pose.pose.position.y = point.y
        self.Best_goal.target_pose.pose.position.z =0
        self.Best_goal.target_pose.pose.orientation.x=self.pose.pose.orientation.x
        self.Best_goal.target_pose.pose.orientation.y=self.pose.pose.orientation.y
        self.Best_goal.target_pose.pose.orientation.z=self.pose.pose.orientation.z
        self.Best_goal.target_pose.pose.orientation.w=self.pose.pose.orientation.w
        self.client.send_goal(self.Best_goal)
        
    
    def callback1(self, odom):
        self.pose=odom.pose
        point=[odom.pose.pose.position.x,odom.pose.pose.position.y]
        indices=self.index_of_point(point)
        for index in indices:
            self.free.add(index)
            self.map.data[index]=0
            
        self.pub1.publish(self.map)


    def getState(self):
        return self.client.get_state()
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