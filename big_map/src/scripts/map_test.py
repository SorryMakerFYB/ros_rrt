#!/usr/bin/env python3 
import rospy 
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32MultiArray
from robot import robot
def map_pub1():
    msg=OccupancyGrid()
    msg.header.frame_id="robot_1/map"
    msg.header.stamp=rospy.Time.now()
    msg.info.width=1000
    msg.info.height=1000
    msg.info.origin.position.x=-500
    msg.info.origin.position.y=-500
    msg.info.resolution=1.0
    msg.data=[-1]*msg.info.width*msg.info.height
    msg.data[0]=0
    pub1.publish(msg)   
def map_pub2():
    msg=OccupancyGrid()
    msg.header.frame_id="robot_2/map"
    msg.header.stamp=rospy.Time.now()
    msg.info.width=1000
    msg.info.height=1000
    msg.info.origin.position.x=-500
    msg.info.origin.position.y=-500
    msg.info.resolution=1.0
    msg.data=[-1]*msg.info.width*msg.info.height
    msg.data[1]=0
    pub2.publish(msg)   
def map_pub3():
    msg=OccupancyGrid()
    msg.header.frame_id="robot_3/map"
    msg.header.stamp=rospy.Time.now()
    msg.info.width=1000
    msg.info.height=1000
    msg.info.origin.position.x=-500
    msg.info.origin.position.y=-500
    msg.info.resolution=1.0
    msg.data=[-1]*msg.info.width*msg.info.height
    msg.data[2]=0
    pub3.publish(msg)   
    
if __name__=="__main__":
    rospy.init_node("fuck_pub")
    pub1=rospy.Publisher("robot_1/map",OccupancyGrid,queue_size=100)
    pub2=rospy.Publisher("robot_2/map",OccupancyGrid,queue_size=100)
    pub3=rospy.Publisher("robot_3/map",OccupancyGrid,queue_size=100)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        map_pub1()
        map_pub2()
        map_pub3()
        rate.sleep()
       

        

       
       

        
