#!/usr/bin/env python3 
import math
import rospy 
from nav_msgs.msg import OccupancyGrid,Odometry
from std_msgs.msg import Int32MultiArray
pub_points=rospy.Publisher("points_around",Int32MultiArray,queue_size=10)
detected=Int32MultiArray()

def radius(x,y):
    freeSpace=set()
    
    r=10
    for i in range(0,1000):
        for j in range(0,1000):
            if((math.pow((i-x),2)+math.pow((j-y),2))<r*r):
                index=j*1000+i
                freeSpace.add(index)
    return freeSpace
def points(odom):
    
    tmp=set(detected.data)
    rospy.loginfo(odom.pose.pose.position.x)
    odom.pose.pose.position.x=odom.pose.pose.position.x
    odom.pose.pose.position.y=odom.pose.pose.position.y
    tmp2=tmp.union(radius(odom.pose.pose.position.x,odom.pose.pose.position.y))
    detected.data=list(tmp2)
    
    pub_points.publish(detected)

# def points(msg):
#     rospy.loginfo("position is %s",msg.pose.pose.position.x)
#     rospy.loginfo("caonima")
if __name__=="__main__":
    rospy.init_node("points_around")
    sub=rospy.Subscriber("odom",Odometry,points,queue_size=10)
    rospy.spin()
    
