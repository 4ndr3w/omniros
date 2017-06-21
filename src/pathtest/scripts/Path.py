import Waypoint
from Segment import Segment
from math import fabs
from nav_msgs.msg import Path as ROSPath
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import Header
import rospy

pointPub = rospy.Publisher('perp', ROSPath, queue_size=10)

class Path:
    def __init__(self, lookaheadDistance):
        self.lookaheadDistance = lookaheadDistance
        self.points = []
        self.segments = []
        self.robot = Waypoint.ORIGIN
        self.lookahead = None
        self.isFinished = False
        self.finishing = False
    
    def rosPath(self):
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = "odom"

        path = ROSPath(header=h)

        first = True
        for p in self.segments:
            if first:
                path.poses.append(PoseStamped(header=h,pose=Pose(position=Point(x=p.start.x, y=p.start.y))))
                first = False
            path.poses.append(PoseStamped(header=h,pose=Pose(position=Point(x=p.stop.x, y=p.stop.y))))
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
        self.robot = robot

        for segment in self.segments[:]:
            perpPoint = segment.closestPointOnPath(robot)

            h = Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "odom"
            
            robotToPathLink = ROSPath(header=h)
            robotToPathLink.poses.append(PoseStamped(header=h,pose=Pose(position=Point(x=robot.x, y=robot.y))))
            robotToPathLink.poses.append(PoseStamped(header=h,pose=Pose(position=Point(x=perpPoint.x, y=perpPoint.y))))
            pointPub.publish(robotToPathLink)

            segment.moveStart(perpPoint)

            if self.finishing:
                if segment.start == segment.stop:
                    self.isFinished = True
                continue


            if segment.distance < self.lookaheadDistance:
                self.lookahead = segment.stop
                if len(self.segments) > 1:
                    self.segments.remove(segment)
                else:
                    # Reaching end on the last segment
                    fakeEnd = segment.start.interpBetween(segment.stop, 2*self.lookaheadDistance+segment.distance)
                    fakeSegment = Segment(segment.start, fakeEnd)
                    self.lookahead = segment.stop.interpBetween(fakeEnd, self.lookaheadDistance)
                    self.finishing = True
                return
            else:
                self.lookahead = perpPoint.interpBetween(segment.stop, self.lookaheadDistance)
            return
