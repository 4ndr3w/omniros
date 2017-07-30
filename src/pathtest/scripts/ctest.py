from Waypoint import Waypoint
from Circle import Circle
from numpy import arange


#circle params
# origin (1, 1)
#radius 1
# start (0, 1)
# stop (1.66472526373, 1.7470878956)

c = Circle(Waypoint(1, 0), 1)
i = 3.2

print(c.pointAt(3.14))
print(c.angleOf(Waypoint(-2, 0)))

#for i in arange(0, 2*3.14, 0.1):
#    print(str(i)+" "+str(c.angleOf(c.pointAt(i))))