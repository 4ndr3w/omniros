#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Path as ROSPath
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from Waypoint import Waypoint
from Path import Path
import math

rospy.init_node('pathpub', anonymous=True)


pub = rospy.Publisher('path', ROSPath, queue_size=1)


rate = rospy.Rate(10) # 10hz

path = Path(1)

path.addWaypoint(Waypoint(0,0))
path.addWaypoint(Waypoint(0,1))
path.addWaypoint(Waypoint(3,3))

origPath = path.rosPath()

def update(odom):
    robot = Waypoint(odom.pose.pose.position.x, odom.pose.position.position.y)
    path.update(robot)
    goal = path.getLookaheadPoint()
    if is not path.isFinished:
        radius = 1 / (2*goal.x)
        maxVel = math.sqrt(2* 5 * path.getRemainingPathLength())
    pub.publish(path.rosPath())
rospy.Subscriber("odom", Odometry, update)

rospy.spin()

