import Waypoint
from Segment import Segment
from math import fabs
from nav_msgs.msg import Path as ROSPath
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import Header
import rospy

class Path:
    def __init__(self, lookaheadDistance):
        self.lookaheadDistance = lookaheadDistance
        self.points = []
        self.segments = []
        self.robot = Waypoint.ORIGIN
        self.lookahead = None
        self.isFinished = False
    
    def rosPath(self):
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "odom"

        path = ROSPath(header=h)
        for p in self.points:
            path.poses.append(PoseStamped(header=h,pose=Pose(position=Point(x=p.x, y=p.y))))
        return path

    def addWaypoint(self, point):
        self.insertPoint(point)
        self.points.append(point)

    def insertPoint(self, point):
        if len(self.points) == 0:
            self.segments.append(Segment(Waypoint.ORIGIN, point))
        else:
            self.segments.append(Segment(self.points[len(self.points)-1], point))

    def reset(self):
        self.isFinished = False
        self.segments = []
        for p in self.points:
            self.insertPoint(p)
    
    def getLookaheadPoint(self):
        return self.lookahead

    def getRemainingPathLength(self):
        length = 0
        for segment in self.segments:
            length = length + segment.distance
        return length

    def update(self, robot):
        epsilon = 0.0001

        self.robot = robot

        # maybe slow
        for segment in self.segments[:]:
            perpPoint = segment.closestPointOnPath(robot)

            distToStart = Waypoint.distance(segment.start, perpPoint)
            distToStop = Waypoint.distance(segment.stop, perpPoint)

            if fabs(segment.distance - (distToStart+distToStart)) > epsilon:
                if distToStart < distToStop:
                    perpPoint = segment.start
                else:
                    perpPoint = segment.stop
                
            segment.moveStart(perpPoint)

            if segment.distance < self.lookaheadDistance:
                if len(self.segments) != 1:
                    self.segments.remove(segment)
                    continue
                else:
                    self.lookahead = segment.stop
                    self.isFinished = True
                    return
            else:
                self.lookahead = perpPoint.interpBetween(segment.stop, self.lookaheadDistance)
            return
