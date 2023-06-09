#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time

pub = rospy.Publisher('pub_with_sub', String, queue_size=10)  #发布话题设为全局变量
def callback(data):
    rospy.loginfo( "I heard %s", data.data)
    pub.publish(data.data)
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("pub1", String, callback,queue_size = 1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()