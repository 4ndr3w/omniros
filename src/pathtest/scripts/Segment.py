from Waypoint import Waypoint, distance
import sys
import numpy

class Segment:
    def __init__(self, start, stop):
        self.start = start
        self.stop = stop

        self.distance = distance(start, stop)

    def moveStart(self, start):
        self.start = start
        self.distance = distance(start, self.stop)
        print("segment moved to "+str(start.x)+" "+str(start.y)+"  new distance is "+str(self.distance))

    def closestPointOnPath(self, p):
        # Handle undefined slope
        if self.start.x == self.stop.x:
            # Closest line intersection is just the vertical line of the path
            # meeting the horizonal line projected by the robot
            return Waypoint(self.start.x, p.y)
        
        segmentSlope = (self.start.y-self.stop.y)/(self.start.x-self.stop.x)

        # Handle 0 slope
        if segmentSlope == 0:
            return Waypoint(p.x, self.start.y)

        segmentIntercept = self.start.y-(segmentSlope*self.start.x)
        perpSlope = -1.0/segmentSlope
        perpIntercept = p.y-(perpSlope*p.x)

        # x=[y;x] 
        A = [[1, -segmentSlope], [1, -perpSlope]]
        B = [segmentIntercept, perpIntercept]

        answer = numpy.linalg.solve(A, B)

        return Waypoint(answer[1], answer[0])