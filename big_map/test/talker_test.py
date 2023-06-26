#!/usr/bin/env python3 
# coding=utf-8
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid


if __name__ == "__main__":
    rospy.init_node("multi_publish")
    pub1 = rospy.Publisher("robot_1/map",OccupancyGrid, queue_size=1)
    map=OccupancyGrid()
    map.header.frame_id="map"
    map.header.stamp=rospy.Time.now()
    map.info.width=10
    map.info.height=10
    #测试起始位置在那里是正中间
    map.info.origin.position.x=-5
    map.info.origin.position.y=-5
    map.info.resolution=1.0
    map.data=[100]*map.info.width*map.info.height
    for i in range(10,20):
         map.data[i]=0
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub1.publish(map)
 
        
        rate.sleep()