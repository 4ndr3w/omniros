#!/usr/bin/env python2
import rospy

from Path import Path
from CircleSegment import CircleSegment
from Waypoint import Waypoint
from nav_msgs.msg import Path as ROSPath

rospy.init_node('pathpub', anonymous=True)

path = Path(1)
path.addWaypoint(Waypoint(0,1), 0)
path.addWaypoint(Waypoint(7,-3), 1)

#path.segments.append(CircleSegment(Waypoint(0,0), 1, 0, 3.14))

pathpub = rospy.Publisher('path', ROSPath, queue_size=10)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    pathpub.publish(path.rosPath())
    rate.sleep()
