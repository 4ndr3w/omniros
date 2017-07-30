from math import sin, cos, atan2, pi, sqrt, fabs
from Waypoint import Waypoint

class Circle:
    def __init__(self, origin, radius):
        self.origin = Waypoint(-origin.x, -origin.y)
        self.radius = fabs(radius)

    def pointAt(self, theta):
        return Waypoint(cos(theta)*self.radius-self.origin.x, sin(theta)*self.radius-self.origin.y)


    def angleOf(self, a):
        print ("angleOf: "+str(Waypoint(a.y + self.origin.y, a.x + self.origin.x)))
        return atan2(a.y + self.origin.y, a.x + self.origin.x)

    def intersection(self, other):
        if isinstance(other, Circle):
            # tX*x + tY*y + tC = tR
            tX = 2*self.origin.x - 2*other.origin.x
            tY = 2*self.origin.y - 2*other.origin.y
            tC = self.origin.x ** 2 + self.origin.y ** 2 - other.origin.x ** 2 - other.origin.y ** 2
            tR = self.radius ** 2 - other.radius ** 2

            # Solve for x
            # Move everything to the right
            tR -= tC
            tY /= -tX
            tR /= tX
            tC = 0
            tX = 1

            # Substitude into circle equation
            # (x-a)^2+(y-b)^2 = r^2

            # Compute (y-b)^2 = r^2 first
            y2 = 1
            y = 2 * self.origin.y
            c = self.origin.y ** 2

            # expand ((tY*y+tR)-a)^2
            tmpC = tR + self.origin.x
            tmpY = tY
            c += tmpC ** 2
            c -= self.radius ** 2

            y2 += tmpY ** 2

            y += 2 * tmpY * tmpC

            solveY1 = (-y + sqrt(y ** 2 - 4 * y2 * c)) / (2 * y2)
            solveY2 = (-y - sqrt(y ** 2 - 4 * y2 * c)) / (2 * y2)

            solveX1 = tY * solveY1 + tR
            solveX2 = tY * solveY2 + tR

            return [
                Waypoint(solveX1, solveY1),
                Waypoint(solveX2, solveY2)
            ]