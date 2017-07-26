from math import atan, sin, cos

class Waypoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return str(self.x)+" "+str(self.y)


x1 = 2
y1 = 2


x2 = 4
y2 = 4

dydx = (y2-y1)/(x2-x1)
#find parallel slope for the thing hopff
t = atan(-1/dydx)

d = 2


print(cos(t))
print(sin(t))

nX = x1+d*cos(t)
nY = y1+d*sin(t)

w = Waypoint(nX,nY)
print(w)


