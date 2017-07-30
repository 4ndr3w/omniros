import numpy
from math import sin,cos,fabs
from geometry_msgs.msg import PoseStamped, Pose, Point
from Circle import Circle

class CircleSegment:
    def __init__(self, origin, radius, entry, exit):
        self.origin = origin
        self.radius = fabs(radius)
        self.entry = entry
        self.exit = exit
        self.circle = Circle(origin, radius)

        print("drawing circle from "+str(self.entry)+" to "+str(self.exit))

    def addToROSPath(self, path, h):
        if self.entry > self.exit:
            r = reversed(numpy.arange(self.exit, self.entry, 0.1))
        else:
            r = numpy.arange(self.entry, self.exit, 0.1)
        
        for a in r:
            path.append(PoseStamped(header=h,pose=Pose(position=Point(x=cos(a)*self.radius+self.origin.x, y=sin(a)*self.radius+self.origin.y))))

    def closestPerpendicularPoint(self, p):
        # Find the angle of the robot to the circle
        theta = self.circle.angleOf(self.origin.translateBy(-origin.x, -origin.y))
        # Find the point on the circle
        p = self.circle.pointAt(theta)
        self.entry = theta

        return p
