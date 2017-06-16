from math import sqrt, pow, fabs, radians, degrees, sin, cos

epsilon = 0.001

def distance(a,b):
    return sqrt(pow(a.x-b.x, 2)+pow(a.y-b.y, 2))

def isBetween(a, b, middle):
    return fabs(distance(a,b) - (distance(a,middle) + distance(middle,b))) < epsilon

class Waypoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def interpBetween(other, lookAhead):
        ratio = lookAhead / distance(self, other)
        return Waypoint(( 1-ratio ) * self.x + ratio*other.x, ( 1-ratio ) * self.y + ratio*other.y)
    
    def rotateBy(theta):
        theta = radians(theta)
        return Waypoint(self.x*cos(theta) - self.y*sin(theta), x*sin(theta) + y*cos(theta))

    def translateBy(dx, dy):
        return Waypoint(self.x+dx, self.y+dy)

ORIGIN = Waypoint(0, 0)