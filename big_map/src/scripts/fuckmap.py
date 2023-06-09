#!/usr/bin/env python3 
import rospy 
from nav_msgs.msg import OccupancyGrid,Odometry
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32MultiArray
from numpy import floor
#def fake_scan(map,pose):
#每次从map_update node订阅到过的地方，遍历涂白再发布地图。
pub=rospy.Publisher("map",OccupancyGrid,queue_size=10)
def index_of_point(mapData, Xp):
    resolution = mapData.info.resolution
    Xstartx = mapData.info.origin.position.x
    Xstarty = mapData.info.origin.position.y
    width = mapData.info.width
    Data = mapData.data
    index = int(	(floor((Xp[1]-Xstarty)/resolution) *
                  width)+(floor((Xp[0]-Xstartx)/resolution)))
    return index
def map_construct(odom):
               
        def around(odom_msg,mapData):
              freespace=set()
              for i in range(-10,10):
                    for j in range(-10,10):
                        point=[odom_msg.pose.pose.position.x+j,odom_msg.pose.pose.position.x+j]
                        freespace.add(index_of_point(mapData,point))
              return freespace
        msg=OccupancyGrid()
        msg.header.frame_id=robot_ns
        msg.header.stamp=rospy.Time.now()
        msg.info.width=1000
        msg.info.height=1000
        #测试起始位置在那里是正中间
        msg.info.origin.position.x=-500
        msg.info.origin.position.y=-500

        msg.info.resolution=1.0
        
        msg.data=[-1]*msg.info.width*msg.info.height
        x=list(around(odom,msg))
        for i in x :
             msg.data[i]=0
            
       
        pub.publish(msg)
        
        

if __name__=="__main__":
    rospy.init_node("map_pub")
    ns=rospy.get_param("~ns",'')
    robot_ns=ns+"/map"
    sub=rospy.SubscribeListener("odom",Odometry,map_construct,queue_size=100)
    rospy.spin()
       
       

        
 # sub=rospy.SubscribeListener('/gazebo/odom',odom,que,queue=)
# def callback(odom):
#     #odom.pose.x
#     #odom.posy.y
#     for i in msg.data:
#         if closeEnough():
#             msg.data[i]=0
