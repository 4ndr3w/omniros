#!/usr/bin/env python2
import rospy

from Queue import PriorityQueue
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from math import floor
import numpy as np

rospy.init_node('astar', anonymous=False)


grid = OccupancyGrid()
grid.header.frame_id = "odom"
grid.info.resolution = 0.05
grid.info.width = 50
grid.info.height = 50
grid.info.origin.position.x = -1
grid.info.origin.position.y = -1

grid.data = np.zeros(grid.info.width*grid.info.height)

def rmo(x, y, len):
    return int(x*len+y)

def rmo2point(idx, len):
    x = floor(idx / len)
    y = idx - (x*len)
    return (int(x), int(y))

for i in range(10):
    if i is 0:
        continue
    grid.data[rmo(i, i, grid.info.width)] = 70

mapPub = rospy.Publisher('map', OccupancyGrid, queue_size=10)
pathPub = rospy.Publisher('path', Path, queue_size=10)



rate = rospy.Rate(1)

delta = [
    [0, 1], # up
    [0, -1], # down
    [1, 0], # left
    [-1, 0], # right

    [1, 1],
    [-1, -1],
    [-1, 1],
    [1, -1]
]

while not rospy.is_shutdown():
    mapPub.publish(grid)
    access = np.zeros(grid.info.width*grid.info.height)
    queue = PriorityQueue()
    cost = [-1 for z in range(grid.info.width*grid.info.height)]

    queue.put([0, 0, 0, 0])
    cost[0] = 0

    goal = [1, 1]

    # Convert goal to map index
    goal[0] -= grid.info.origin.position.x
    goal[1] -= grid.info.origin.position.y

    goal[0] /= grid.info.resolution
    goal[1] /= grid.info.resolution

    goal[0] = int(goal[0])
    goal[1] = int(goal[1])

    found = False
    while not queue.empty():
        point = queue.get()

        if point[1] is goal[0] and point[2] is goal[1]:
            found = True
            break

        idx = rmo(point[1], point[2], grid.info.width)
        newCost = cost[idx] + 1
        for d in delta:
            newPoint = [point[0]+1, point[1]+d[0], point[2]+d[1]]
            newIndex = rmo(newPoint[1],newPoint[2], grid.info.width)
            if newPoint[1] < 0 or newPoint[2] < 0 or newPoint[1] >= grid.info.width or newPoint[2] >= grid.info.height:
                continue
            # Blocked
            if grid.data[idx] > 50:
                cost[newIndex] = 10000
                continue
            
            if (cost[newIndex] == -1 or newCost < cost[newIndex]):
                cost[newIndex] = newCost
                # Got here via orig point
                access[newIndex] = rmo(point[1],point[2], grid.info.width)
                queue.put(newPoint)
                
    if found:
        print("yay!")
        # Convert the path we found into a ROS path
        start = rmo(0, 0, grid.info.width)
        end = rmo(goal[0], goal[1], grid.info.width)
        pos = end
        hops = []
        while pos != start:
            hops.append(pos)
            pos = access[pos]
        hops.append(start)
        hops.reverse()

        path = Path()
        path.header.frame_id = "odom"
        path.poses = []
        for hop in hops:
            pose = PoseStamped()
            pose.header.frame_id = "odom"

            x, y = rmo2point(hop, grid.info.width)

            # Convert back to ROS coordinates
            x *= grid.info.resolution
            y *= grid.info.resolution

            x += grid.info.origin.position.x
            y += grid.info.origin.position.y

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0
            path.poses.append(pose)
        pathPub.publish(path)

    else:
        path = Path()
        path.header.frame_id = "odom"
        path.poses = []
        pathPub.publish(path)
        print("nay")


    m = np.reshape(grid.data, (grid.info.height, grid.info.width))
    
    rate.sleep()
