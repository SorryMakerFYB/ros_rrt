#!/usr/bin/env python3


#--------Include modules---------------
import rospy
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped,Point
from getfrontier import getfrontier
from sklearn.cluster import KMeans,MeanShift
from copy import copy
from big_map.msg import PointArray
#-----------------------------------------------------
# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()


def mapCallBack(data):
    global mapData
    mapData=data
    

    

# Node----------------------------------------------
def node():
		global mapData
		exploration_goal=Point()
		rospy.init_node('detector', anonymous=False)
		map_topic= rospy.get_param('~map_topic','/robot_1/map')
		color_r=rospy.get_param('~color_r',255)
		color_g=rospy.get_param('~color_g',0)
		color_b=rospy.get_param('~color_b',0)
		rospy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
		targetspub = rospy.Publisher('detected_points', PointArray, queue_size=10)
		pub = rospy.Publisher('shapes', Marker, queue_size=10)
		
# wait until map is received, when a map is received, mapData.header.seq will not be < 1
		while mapData.header.seq<1 or len(mapData.data)<1:
			pass
    	   	
		rate = rospy.Rate(1)	
		points=Marker()

		#Set the frame ID and timestamp.  See the TF tutorials for information on these.
		points.header.frame_id=mapData.header.frame_id
		points.header.stamp=rospy.Time.now()

		points.ns= "markers"
		points.id = 0

		points.type = Marker.POINTS
		#Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		points.action = Marker.ADD

		points.pose.orientation.w = 1.0
		points.scale.x=points.scale.y=1
		points.color.r = color_r/255.0
		points.color.g = color_g/255.0
		points.color.b = color_b/255.0
		points.color.a=1
		points.lifetime == rospy.Duration()

#-------------------------------OpenCV frontier detection------------------------------------------
		while not rospy.is_shutdown():
			frontiers=getfrontier(mapData)
			k=KMeans(10).fit(frontiers)
			#frontiers=k.cluster_centers_

			points.points=[]
			all_points=[]
			for i in range(len(frontiers)):
				x=frontiers[i]
				if abs(x[0])>100 or abs(x[0])>100:
					pass
				else:
					exploration_goal.x=x[0]
					exploration_goal.y=x[1]
					exploration_goal.z=0	
					points.points.append(copy(exploration_goal))	
					all_points.append(copy(exploration_goal))	
			targetspub.publish(all_points)
			
			pub.publish(points) 
			rate.sleep()
          	
		

	  	#rate.sleep()



#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass
 
 
 
 
