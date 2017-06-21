from Waypoint import Waypoint, distance, isBetween
import sys
import numpy
from math import fabs

class Segment:
    def __init__(self, start, stop):
        self.start = start
        self.stop = stop

        self.distance = distance(start, stop)

    def moveStart(self, start):
        self.start = start
        self.distance = distance(start, self.stop)
        #print("segment moved to "+str(start.x)+" "+str(start.y)+"  new distance is "+str(self.distance))

    def closestPointOnPath(self, p):
        perpPoint = self.closestPerpendicularPoint(p)
        # Clamp to segment
        #if isBetween(self.start, self.stop, perpPoint) == False:
        #    if distance(self.start, perpPoint) < distance(self.stop, perpPoint):
        #        perpPoint = self.start
        #    else:
        #       perpPoint = self.stop

        d2Start = distance(self.start, perpPoint)
        d2Stop = distance(self.stop, perpPoint)

        if fabs(self.distance - (d2Start+d2Stop)) > 0.00001:
            if d2Start < d2Stop:
                return self.start
            else:
                return self.stop
        return perpPoint

    def closestPerpendicularPoint(self, p):
        if self.start.x == self.stop.x and self.start.y == self.stop.y:
            return self.stop

        # Handle undefined slope
        if self.start.x == self.stop.x:
            # Closest line intersection is just the vertical line of the path
            # meeting the horizonal line projected by the robot
            return Waypoint(self.start.x, p.y)
        
        segmentSlope = float(self.start.y-self.stop.y)/float(self.start.x-self.stop.x)

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