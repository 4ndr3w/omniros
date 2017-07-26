from math import sqrt, pow, fabs, radians, degrees, sin, cos

def distance(a,b):
    return sqrt(pow(a.x-b.x, 2)+pow(a.y-b.y, 2))

def midpoint(a,b):
    return Waypoint((a.x+b.x)/2.0, (a.y+b.y)/2.0)

def isBetween(a, b, c):
    "Return true iff point c intersects the line segment from a to b."
    return (collinear(a, b, c)
            and (within(a.x, c.x, b.x) if a.x != b.x else 
                 within(a.y, c.y, b.y)))

def collinear(a, b, c):
    "Return true iff a, b, and c all lie on the same line."
    return (b.x - a.x) * (c.y - a.y) == (c.x - a.x) * (b.y - a.y)

def within(p, q, r):
    "Return true iff q is between p and r (inclusive)."
    return p <= q <= r or r <= q <= p

class Waypoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    
    def interpBetween(self, other, lookAhead):
        ratio = lookAhead / distance(self, other)
        return Waypoint(( 1-ratio ) * self.x + ratio*other.x, ( 1-ratio ) * self.y + ratio*other.y)
    
    def rotateBy(self, theta):
        return Waypoint(self.x*cos(theta) - self.y*sin(theta), self.x*sin(theta) + self.y*cos(theta))

    def translateBy(self, dx, dy):
        return Waypoint(self.x+dx, self.y+dy)


    def __str__(self):
        return "("+str(self.x)+", "+str(self.y)+")"

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

ORIGIN = Waypoint(0, 0)