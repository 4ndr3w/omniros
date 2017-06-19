#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Path as ROSPath
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from Waypoint import Waypoint, distance
from Path import Path
from math import asin, degrees
import math

rospy.init_node('pathpub', anonymous=True)


pathpub = rospy.Publisher('path', ROSPath, queue_size=10)
pub = rospy.Publisher('cmd_velocity', Twist, queue_size=5)

rate = rospy.Rate(10) # 10hz

path = Path(0.5)

path.addWaypoint(Waypoint(0,2))
path.addWaypoint(Waypoint(3,3))

origPath = path.rosPath()

def update(odom):
    robot = Waypoint(odom.pose.pose.position.x, odom.pose.pose.position.y)

    path.update(robot)
    goal = path.getLookaheadPoint()
    q = odom.pose.pose.orientation
    yaw = asin(2*q.x*q.y + 2*q.z*q.w) + 3.14/2
    print("robot at "+str(odom.pose.pose.position.x)+" "+str(odom.pose.pose.position.y)+" heading "+str(degrees(yaw)))
    goal = goal.translateBy(-robot.x, -robot.y).rotateBy(-yaw)
    if path.isFinished == False:
        radius = 0
        if goal.x != 0:
            radius = Waypoint.distance(robot, goal) / (2*goal.x)
        maxVel = math.sqrt(2* 1 * path.getRemainingPathLength())
        print("remaining "+str(path.getRemainingPathLength())+" goal at "+str(goal.x)+" "+str(goal.y))
        omega = 0
        if radius != 0:
            omega = maxVel / radius
        print("vel "+str(maxVel)+" omega "+str(omega))
        pub.publish(Twist(linear=Vector3(x=0, y=maxVel, z=0), angular=Vector3(x=0, y=0, z=-omega)))
    else:
        pub.publish(Twist(linear=Vector3(x=0, y=0, z=0), angular=Vector3(x=0, y=0, z=0)))

    pathpub.publish(path.rosPath())
rospy.Subscriber("odom", Odometry, update)

rospy.spin()

