#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Path as ROSPath
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
from Waypoint import Waypoint, distance
from Path import Path
from math import asin, degrees, atan2,atan
import math
import tf

rospy.init_node('pathpub', anonymous=True)

origpub = rospy.Publisher('orig', ROSPath, queue_size=10)
pathpub = rospy.Publisher('path', ROSPath, queue_size=10)
linkpub = rospy.Publisher('link', ROSPath, queue_size=10)
pub = rospy.Publisher('cmd_velocity', Twist, queue_size=5)

lookahead = 1

path = Path(lookahead)

path.addWaypoint(Waypoint(0,2))
path.addWaypoint(Waypoint(-2, 4))
path.addWaypoint(Waypoint(3,6))
path.addWaypoint(Waypoint(-3,6))
path.addWaypoint(Waypoint(2,0))
path.addWaypoint(Waypoint(0,0))

# Gear auto
path.addWaypoint(Waypoint(0,2))
path.addWaypoint(Waypoint(2,4))

origPath = path.rosPath()

vel = 0
lastTime = rospy.Time.now() 

def update(odom):
    global vel
    global lastTime
    maxAcc = 1

    dt = (rospy.Time.now()-lastTime).to_sec()
    lastTime = rospy.Time.now()

    robot = Waypoint(odom.pose.pose.position.x, odom.pose.pose.position.y)

    path.update(robot)
    goal = path.getLookaheadPoint()

    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = "odom"

    robotToPathLink = ROSPath(header=h)
    robotToPathLink.poses.append(PoseStamped(header=h,pose=Pose(position=odom.pose.pose.position)))
    robotToPathLink.poses.append(PoseStamped(header=h,pose=Pose(position=Point(x=goal.x, y=goal.y))))
    linkpub.publish(robotToPathLink)


    q = odom.pose.pose.orientation
    (r, p, yaw) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

    goal = goal.translateBy(-robot.x, -robot.y).rotateBy(-yaw)
    if path.isFinished == False:
        radius = 0

        if goal.x != 0:
            radius = lookahead / (2*goal.x)
        maxVel = math.sqrt(2 * maxAcc * path.getRemainingPathLength())

        vel += maxAcc * dt

        if vel > maxVel:
            vel = maxVel
        
        if vel > 2:
            vel = 2

        omega = 0
        if radius != 0:
            omega = maxVel / radius
        pub.publish(Twist(linear=Vector3(x=0, y=vel, z=0), angular=Vector3(x=0, y=0, z=-omega)))
    else:
        pub.publish(Twist(linear=Vector3(x=0, y=0, z=0), angular=Vector3(x=0, y=0, z=0)))
        rospy.signal_shutdown("Task complete")

    pathpub.publish(path.rosPath())
    origpub.publish(origPath)
rospy.Subscriber("odom", Odometry, update)

rospy.spin()

