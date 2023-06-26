#!/usr/bin/env python3


#--------Include modules---------------
from copy import copy
import rospy
from nav_msgs.msg import OccupancyGrid
from numpy import floor,array
import numpy as np
import cv2
from sklearn.cluster import k_means

#-----------------------------------------------------

def getfrontier(mapData):
	data=mapData.data
	w=mapData.info.width
	h=mapData.info.height
	resolution=mapData.info.resolution
	Xstartx=mapData.info.origin.position.x
	Xstarty=mapData.info.origin.position.y
	def point_of_index(mapData, i):
		y = mapData.info.origin.position.y + \
        (i//mapData.info.width)*mapData.info.resolution
		x = mapData.info.origin.position.x + \
        (i-(i//mapData.info.width)*(mapData.info.width))*mapData.info.resolution
		return array([x, y])
	
	img = np.zeros((h, w, 1), np.uint8)
	
	for i in range(0,h):
		for j in range(0,w):
			if data[i*w+j]==0:
				img[i,j]=0
			elif data[i*w+j]==100:
				img[i,j]=255
			elif data[i*w+j]==-1:
				img[i,j]=205
	
	
	all_pts=[]
	edges = cv2.Canny(img,100,200)
	for i in range(0,edges.shape[0]):
		for j in range(0,edges.shape[1]):
			if edges[i,j]>100:
				index=i*w+j
				all_pts.append(point_of_index(mapData,index))
				
				
	



	
	return all_pts

