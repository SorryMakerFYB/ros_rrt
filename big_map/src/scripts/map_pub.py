#!/usr/bin/env python3 
import rospy 
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32MultiArray
#def fake_scan(map,pose):
#每次从map_update node订阅到过的地方，遍历涂白再发布地图。
pub=rospy.Publisher("map",OccupancyGrid,queue_size=10)
def map_construct(points):
        msg=OccupancyGrid()
        msg.header.frame_id=robot_ns
        msg.header.stamp=rospy.Time.now()
        msg.info.width=1000
        msg.info.height=1000
        #测试起始位置在那里是正中间
        msg.info.origin.position.x=0
        msg.info.origin.position.y=0

        msg.info.resolution=1.0
        
        msg.data=[-1]*msg.info.width*msg.info.height
        
        for i in points.data :
             msg.data[i]=0
        pub.publish(msg)
        
        

if __name__=="__main__":
    rospy.init_node("map_pub")
    ns=rospy.get_param("~ns",'')
    robot_ns=ns+"/map"
    sub=rospy.Subscriber("points_around",Int32MultiArray,map_construct,queue_size=1000)
    
    rospy.spin()
       
       

        
 # sub=rospy.SubscribeListener('/gazebo/odom',odom,que,queue=)
# def callback(odom):
#     #odom.pose.x
#     #odom.posy.y
#     for i in msg.data:
#         if closeEnough():
#             msg.data[i]=0
