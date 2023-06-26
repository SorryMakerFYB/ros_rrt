#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
timestart=rospy.Time(0)
timeend=rospy.Time(0)
def talker():
    pub = rospy.Publisher('pub1', String, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    timestart=rospy.Time.now()
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        timeend=rospy.Time.now()
        rospy.loginfo(timeend.secs-timestart.secs>10)
        pub.publish(hello_str)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass