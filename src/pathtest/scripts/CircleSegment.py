import numpy
from math import sin,cos,fabs
from geometry_msgs.msg import PoseStamped, Pose, Point

class CircleSegment:
    def __init__(self, origin, radius, entry, exit):
        self.origin = origin
        self.radius = fabs(radius)
        self.entry = entry
        self.exit = exit

        print("drawing circle from "+str(self.entry)+" to "+str(self.exit))

    def addToROSPath(self, path, h):
        if self.entry > self.exit:
            r = reversed(numpy.arange(self.exit, self.entry, 0.1))
        else:
            r = numpy.arange(self.entry, self.exit, 0.1)
        
        for a in r:
            path.append(PoseStamped(header=h,pose=Pose(position=Point(x=cos(a)*self.radius+self.origin.x, y=sin(a)*self.radius+self.origin.y))))
