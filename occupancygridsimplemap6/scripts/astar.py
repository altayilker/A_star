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

delta = [[-1, 0], # go up
        [ 0,-1], # go left
        [ 1, 0], # go down
        [ 0, 1]] # go right

delta_name = ['^', '<', 'v', '>']


def heuristic(goal, current_x, current_y):

    x = abs(goal[0] - current_x) 
    y = abs(goal[1] - current_y)

    return x + y 


def search(grid,init,goal,cost):
    # ----------------------------------------
    # modify the code below
    # ----------------------------------------
    closed = [[0 for col in range(len(grid[0]))] for row in range(len(grid))]
    closed[init[0]][init[1]] = 1

    expand = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]
    action = [[-1 for col in range(len(grid[0]))] for row in range(len(grid))]


    """
    heuristic = [[9, 8, 7, 6, 5, 4],
                [8, 7, 6, 5, 4, 3],
                [7, 6, 5, 4, 3, 2],
                [6, 5, 4, 3, 2, 1],
                [5, 4, 3, 2, 1, 0]]
    """

    x = init[0]
    y = init[1]
    g = 0
    f = g + heuristic(goal, x, y)


    open = [[f, g, x, y]]

    found = False  # flag that is set when search is complete
    resign = False # flag set if we can't find expand
    count = 0
    
    while not found and not resign:
        if len(open) == 0:
            resign = True
            print("Fail")
            return "Fail"
        else:
            open.sort()
            open.reverse()
            next = open.pop()
            x = next[2]
            y = next[3]
            f = next[0]
            g = next[1]
            expand[x][y] = count
            count += 1
            
            if x == goal[0] and y == goal[1]:
                found = True
                print("we reach the goal")
                print("goal is :" , [x, y] )
                # action[x][y] = '*'

            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            f = g2 + heuristic(goal, x2, y2)
                            # print("heuristic :", heuristic(goal, x2, y2))
                            open.append([f, g2, x2, y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i
                            # print("action[x][y] : " , action[x][y])
                            # print("action :", action)
                            # input("enter tusuna basiniz")


    for i in range(len(expand)):
        print("expand: ", expand[i])
    
    for i in range(len(action)):
        print("action :", action[i])


    policy = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]
    policy_numbers = [[' ' for col in range(len(grid[0]))] for row in range(len(grid))]

    x = goal[0]
    y = goal[1]
    policy[x][y] = '*'

    count = 0
    policy_iter_list = []
    while x != init[0] or y != init[1]:
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        policy[x2][y2] = delta_name[action[x][y]]
        count += 1
        policy_iter_list.append([count, x, y]) 
        policy_numbers[x2][y2]  = action[x][y]
        

        x = x2
        y = y2

    for i in range(len(policy)):
        print("policy : ", policy[i])

    for i in range(len(policy)):
        print("policy_numbers : ", policy_numbers[i])

    policy_iter_list.append([count+1, init[0], init[1]]) 
    for i in range(len(policy_iter_list)):
        print("policy_iter_list : ", policy_iter_list[i])

    policy_iter_list.sort(reverse=True)
    print("policy_iter_list : ", policy_iter_list)

    return policy_iter_list # expand


def find_point_in_occupancygrid(x, y, resolution, width, height, x2, y2):
    y_grid = math.floor((x- x2)/resolution)  
    x_grid = math.floor((y- y2)/resolution)
    print("x_grid : ", y_grid)
    print("y_grid : ", x_grid)
    print("x_grid + y_grid : ", x_grid + y_grid)
    return x_grid , y_grid, (height*width) -1


def convert_occupancygrid_to_point(occupancy_number, resolution, occupancy_grid):
    x = (occupancy_number % occupancy_grid.info.width)*resolution 
    y = (occupancy_number / occupancy_grid.info.width)*resolution 
    print("occupancy_number : ", occupancy_number)
    print("occupancy_grid.info.width : ", occupancy_grid.info.width)
    # print("occupancy_grid : ", occupancy_grid)

    print("pointx : ", x)
    print("pointy : ", y)
    return x, y



def sample_deko(occupancy_grid):

     
    rate = rospy.Rate(1) # 10hz

    markerArray_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size = 2)
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)

    count = 0
    counter = 0
    counter2 = 0

    # init = [0, 0]
    # goal = [4, 5]
    cost = 1




    while not rospy.is_shutdown():
        now = rospy.Time.now()
        markerArray = MarkerArray()

   
        
        # read init pose
        if counter >= 0:
            msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped, timeout=10)
            print("mesaj okundu")
            counter += 1

        print("marker.pose.position.x :", msg.pose.pose.position.x)
        print("marker.pose.position.y :", msg.pose.pose.position.y)
        occupancy_numberx, occupancy_numbery, occupancy_number  = find_point_in_occupancygrid(msg.pose.pose.position.x, msg.pose.pose.position.y, occupancy_grid.info.resolution, occupancy_grid.info.width, occupancy_grid.info.height, occupancy_grid.info.origin.position.x, occupancy_grid.info.origin.position.y)
        print("occupancy number : " , occupancy_number)
        print("occupancy numberx : " , occupancy_numberx)
        print("occupancy numbery : " , occupancy_numbery)
        init = [occupancy_numberx, occupancy_numbery]

        # pointx, pointy = convert_occupancygrid_to_point(occupancy_number, occupancy_grid.info.resolution, occupancy_grid)

        
        # read goal pose
        if counter2 >= 0:
            msg2 = rospy.wait_for_message('/move_base_simple/goal', PoseStamped, timeout=10)
            print("mesaj okundu")
            counter2 += 1
        print("marker.pose.position.x :", msg2.pose.position.x)
        print("marker.pose.position.y :", msg2.pose.position.y)
        occupancy_numberx, occupancy_numbery, occupancy_number  = find_point_in_occupancygrid(msg2.pose.position.x, msg2.pose.position.y, occupancy_grid.info.resolution, occupancy_grid.info.width, occupancy_grid.info.height)
        print("occupancy number : " , occupancy_number)
        print("occupancy numberx : " , occupancy_numberx)
        print("occupancy numbery : " , occupancy_numbery)
        goal = [occupancy_numberx, occupancy_numbery]
        
        

        
        # create the grid
        count = 0
        rows, cols = (occupancy_grid.info.height, occupancy_grid.info.width)
        grid = [[0 for i in range(cols)] for j in range(rows)]       
        for i in range(rows):
            for j in range(cols):
                grid[i][j] = occupancy_grid.data[count]
                count += 1
             
        print("grid: " , grid)
        print("init: " , init)
        print("goal: " , goal)
        print("cost: " , cost)

        policy_iter_list = search(grid,init,goal,cost)
        print("policy_iter_list : ", policy_iter_list)



        # count = 0
        for i in range(len(policy_iter_list)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "my_namespace"
            marker.id = i
            marker.type = 1
            marker.action = 0
            marker.pose.position.x = occupancy_grid.info.resolution/2 + occupancy_grid.info.origin.position.x + policy_iter_list[i][2]*occupancy_grid.info.resolution
            marker.pose.position.y = occupancy_grid.info.resolution/2 + occupancy_grid.info.origin.position.y + policy_iter_list[i][1]*occupancy_grid.info.resolution
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = occupancy_grid.info.resolution
            marker.scale.y = occupancy_grid.info.resolution
            marker.scale.z = 0.01
            marker.color.a = 1.0  #Don't forget to set the alpha!
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime = rospy.Duration()
            markerArray.markers.append(marker)
            markerArray_pub.publish(markerArray)
            count += 0.1
            rate.sleep()
        
        
                

        print("marker printing")


def sample_demo():
    
    rospy.Subscriber("/occupancy_grid", OccupancyGrid, sample_deko)
    rospy.spin()
           


if __name__ == "__main__":
    rospy.init_node("test_astar")
    sample_demo()

  
