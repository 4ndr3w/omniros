import Waypoint
from Circle import Circle
from Segment import Segment
from CircleSegment import CircleSegment
from math import fabs, atan
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

        for p in self.segments:
            p.addToROSPath(path.poses, h)
        return path 

    def addWaypoint(self, point, incomingRadius):
        self.insertPoint(point, incomingRadius)
        self.points.append(point)

    def insertPoint(self, point, incomingRadius):
        incomingRadius = abs(incomingRadius)

        if len(self.points) == 0:
            self.segments.append(Segment(Waypoint.ORIGIN, point))
        else:
            # Find the circle's center point using the previous segment
            prev = self.segments[-1]
            # Perp slope to last segment
            slope = prev.slope()
            circleOrigin = None
            if slope is not None:
                theta = atan(-1 / prev.slope())
                # Circle origin
                circleOrigin = Waypoint.Waypoint(prev.stop.x+incomingRadius*cos(theta), prev.stop.y+incomingRadius*sin(theta))
            # undefined slope (vertical)
            else:
                if point.x < prev.stop.x:
                    incomingRadius = -incomingRadius
                circleOrigin = Waypoint.Waypoint(prev.stop.x+incomingRadius, prev.stop.y)

            turnCircle = Circle(circleOrigin, incomingRadius)
            # Create a bigger circle between the new circle midpoint and the end point
            # to find tangents from the endpoint and the circle
            tangentCircleOrigin = Waypoint.midpoint(circleOrigin, point)
            tangentCircleRadius = Waypoint.distance(circleOrigin, point)/2.0
            tangentCircle = Circle(tangentCircleOrigin, tangentCircleRadius)
            # The intersection points of this circle and the turning radius circle create lines
            # which are tangent to the new point from the turning radius
            

            tangentPoint = tangentCircle.intersection(turnCircle)[0]
            print("circle params")
            print(circleOrigin)
            print(incomingRadius)
            print(prev.stop)
            print(tangentPoint)


            self.segments.append(CircleSegment(circleOrigin, incomingRadius, turnCircle.angleOf(prev.stop), turnCircle.angleOf(tangentPoint)))
            #self.segments.append(CircleSegment(circleOrigin, incomingRadius, 0.8, 1.1))
            #self.segments.append(CircleSegment(tangentCircleOrigin, tangentCircleRadius, 0, 2*3.14))
            

            self.segments.append(Segment(tangentPoint, point))

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

            if len(self.segments) == 1:
                if segment.start == segment.stop:
                    self.isFinished = True

            if segment.distance < self.lookaheadDistance:
                self.lookahead = segment.stop
                if len(self.segments) > 1 and segment.stop == segment.start:
                    self.segments.remove(segment)
                return
            else:
                self.lookahead = perpPoint.interpBetween(segment.stop, self.lookaheadDistance)
            return
