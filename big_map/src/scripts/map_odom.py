#! /usr/bin/env python3

# 1.导包
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid,Odometry
def dopose(pose):
     # 3.创建 坐标广播器
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    # 4.创建并组织被广播的消息
    tfs = TransformStamped()
    # --- 头信息
    tfs.header.frame_id = "map"
    tfs.header.stamp = rospy.Time.now()
    
    # --- 子坐标系
    tfs.child_frame_id = child_frame_id_pub
    # --- 坐标系相对信息
    # ------ 偏移量
    tfs.transform.translation.x = 0
    tfs.transform.translation.y = 0
    tfs.transform.translation.z = 0.0
    # ------ 四元数
    qtn = tf.transformations.quaternion_from_euler(0,0,0)
   
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]


    # 5.广播器发送消息
    broadcaster.sendTransform(tfs)
if __name__ == "__main__":
    # 2.初始化 ROS 节点
    rospy.init_node("static_tf_pub_p")
    frame_id= rospy.get_param('~frame_id','')
    child_frame_id= rospy.get_param("~child_frame_id",'')
    child_frame_id_pub=child_frame_id+"/odom"
    sub=rospy.Subscriber("odom",Odometry,dopose)
    
   
    # 6.spin
    rospy.spin()
