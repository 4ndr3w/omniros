#!/usr/bin/env python2
import rospy

from Queue import PriorityQueue
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist, Vector3
import numpy as np
import atf
import tf
import math

rospy.init_node('path', anonymous=False)

lookahead = 1
maxVel = 1
maxAcc = 1

vel = 0
lastTime = rospy.Time.now() 

cmdvel = rospy.Publisher('cmd_velocity', Twist, queue_size=5)

robotLocation = None
path = None

def handleNewPath(newPath):
    global path
    if robotLocation is None:
        print("Got path, but no robot pose")
        return

    if len(newPath.poses) is 0:
        print("Empty path")
        return

    robot = robotLocation.pose.pose.position
    closest = None
    for idx, p in enumerate(newPath.poses):
        dist = atf.distance(robot, p.pose.position)
        if closest is None or dist < closest[1]:
            closest = (idx, dist)

    path = newPath.poses[closest[0]:len(newPath.poses)]


def update(odom):
    global vel
    global lastTime
    global robotLocation
    robotLocation = odom
    robot = robotLocation.pose.pose.position
    if path is None:
        return

    closest = None
    lookaheadPoint = None
    lenPath = len(path)

    remainingDistance = atf.distance(robot, path[lenPath-1].pose.position)

    for idx, p in enumerate(path):
        dist = atf.distance(robot, p.pose.position)
        if closest is None or dist < closest[1]:
            closest = (idx, dist, p)
    
    for p in path[closest[0]:lenPath]:
        if atf.distance(closest[2].pose.position, p.pose.position) >= lookahead:
            lookaheadPoint = p
            break
    
    if lookaheadPoint is None:
        lookaheadPoint = path[lenPath-1]

    q = odom.pose.pose.orientation
    r, p, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    goal = atf.transformBy(lookaheadPoint.pose.position, atf.inverse(robot))
    goal = atf.rotateBy(goal, -yaw)

    maxAllowedVel = math.sqrt(2 * maxAcc * remainingDistance)

    dt = (rospy.Time.now()-lastTime).to_sec()
    lastTime = rospy.Time.now()

    vel += maxAcc * dt

    if vel > maxAllowedVel:
        vel = maxAllowedVel

    if vel > maxVel:
        vel = maxVel

    omega = 0
    if goal.x is not 0:
        omega = vel / (lookahead / (2*goal.x))
    if remainingDistance > 0.01:
        cmdvel.publish(Twist(linear=Vector3(x=0, y=vel, z=0), angular=Vector3(x=0, y=0, z=-omega)))
    else:
        cmdvel.publish(Twist(linear=Vector3(x=0, y=0, z=0), angular=Vector3(x=0, y=0, z=0)))


rospy.Subscriber("odom", Odometry, update)
rospy.Subscriber("path", Path, handleNewPath)
rospy.spin()