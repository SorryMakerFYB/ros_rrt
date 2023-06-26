#!/usr/bin/env python3 
# coding=utf-8
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
pub1 = rospy.Publisher("robot_1/map",OccupancyGrid, queue_size=1)
def call(points):
    map=OccupancyGrid()
    map.header.frame_id="map"
    map.header.stamp=rospy.Time.now()
    map.info.width=10
    map.info.height=10
    #测试起始位置在那里是正中间
    map.info.origin.position.x=-10
    map.info.origin.position.y=-10
    map.info.resolution=2.0
    map.data=[100]*map.info.width*map.info.height
    map.data[0]=0
    map.data[5]=0
    map.data[55]=0
    map.data[53]=0
    
    pub1.publish(map)
if __name__ == "__main__":
    rospy.init_node("multi_publish")
    rospy.Subscriber("detected_points",Float32MultiArray , call)
    
    rospy.spin()