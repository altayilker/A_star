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


def sample_demo():
    
    
    occupancy_pub = rospy.Publisher("/occupancy_grid", OccupancyGrid,  queue_size=1)
  
    rate = rospy.Rate(1) # 10hz

 
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        occupancy_grid = OccupancyGrid()


        occupancy_grid.header.frame_id = "map"
        occupancy_grid.header.stamp = rospy.Time.now()
        occupancy_grid.info.map_load_time = now
        occupancy_grid.info.resolution = 0.3 
        occupancy_grid.info.width = 6
        occupancy_grid.info.height = 5
        occupancy_grid.info.origin.position.x = -1
        occupancy_grid.info.origin.position.y = -1
        occupancy_grid.info.origin.position.z = 0
        occupancy_grid.info.origin.orientation.x = 0
        occupancy_grid.info.origin.orientation.y = 0
        occupancy_grid.info.origin.orientation.z = 0
        occupancy_grid.info.origin.orientation.w = 0
        # occupancy_grid.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 100, 100, 100, 0, 0, 0, 0, 0, 0, 0, 100, 100, 100, 0, 0, 0, 0, 0] 
        occupancy_grid.data = [0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 100, 0, 0, 0, 100, 0, 100, 0, 0, 0, 100, 0, 100, 0] 
        
        occupancy_pub.publish(occupancy_grid)

             
        rate.sleep()
        

       
if __name__ == "__main__":
    rospy.init_node("test_occupancy_grid")
    sample_demo()


  
