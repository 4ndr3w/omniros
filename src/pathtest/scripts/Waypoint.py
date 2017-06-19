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
    
    def interpBetween(self, other, lookAhead):
        ratio = lookAhead / distance(self, other)
        return Waypoint(( 1-ratio ) * self.x + ratio*other.x, ( 1-ratio ) * self.y + ratio*other.y)
    
    def rotateBy(self, theta):
        #theta = radians(theta)
        return Waypoint(self.x*cos(theta) - self.y*sin(theta), self.x*sin(theta) + self.y*cos(theta))

    def translateBy(self, dx, dy):
        return Waypoint(self.x+dx, self.y+dy)

    def __str__(self):
        return "("+str(self.x)+", "+str(self.y)+")"

ORIGIN = Waypoint(0, 0)