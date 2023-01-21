#!/usr/bin/env python

from __future__ import print_function


import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, Vector3, PoseWithCovarianceStamped, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

import random
import numpy as np
import tf
import math




def callback(mapdata):
    
    
    occupancy_pub = rospy.Publisher("/occupancy_grid", OccupancyGrid,  queue_size=100)
  
    rate = rospy.Rate(10) # 10hz
    count = 0


 
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        occupancy_grid = OccupancyGrid()


        occupancy_grid.header.frame_id = "map"
        occupancy_grid.header.stamp = rospy.Time.now()
        occupancy_grid.info.map_load_time = now
        occupancy_grid.info.resolution = 0.05 
        occupancy_grid.info.width =  384
        occupancy_grid.info.height =  384
        occupancy_grid.info.origin.position.x = 0
        occupancy_grid.info.origin.position.y = 0
        occupancy_grid.info.origin.position.z = 0
        occupancy_grid.info.origin.orientation.x = 0
        occupancy_grid.info.origin.orientation.y = 0
        occupancy_grid.info.origin.orientation.z = 0
        occupancy_grid.info.origin.orientation.w = 1
        # occupancy_grid.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0, 100, 100, 100, 0, 0, 0, 0, 0] 
        # occupancy_grid.data = [0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 100, 0, 0, 0, 100, 0, 100, 0, 0, 0, 100, 0, 100, 0] 
        # free (0), occupied (100), and unknown (-1).

        # print("len bu : ", len(occupancy_grid.data))
        

        # print("Reafdgdfg")
        # print("len(map.data) :", len(mapdata.data))
        occupancy_grid.data = list(mapdata.data[0:147456]) # [0:147456]
        # print("occupancy_grid.data  :", occupancy_grid.data )

        
        for i in range(len(occupancy_grid.data)):
            if occupancy_grid.data[i] == -1:
                occupancy_grid.data[i] = 0
            # elif occupancy_grid.data[i] == 100:
            #    occupancy_grid.data[i] = 1
        # print("occupancy_grid.data  :", occupancy_grid.data )

        # print("alist :", aList)
        # print("len(occupancy_grid.data) :", len(occupancy_grid.data))
        # print("haritanin bir araligi :", occupancy_grid.data[40000:82000])
        occupancy_pub.publish(occupancy_grid)

        rate.sleep()
        

       
if __name__ == "__main__":
    rospy.init_node("test_occupancy_grid")
    r = rospy.Subscriber("/map", OccupancyGrid,  callback)
    rospy.spin()


  