from Waypoint import Waypoint
from Circle import Circle

c1 = Circle(Waypoint(0, 0), 3)
c2 = Circle(Waypoint(1,1), 3)

i = c1.intersection(c2)

print(i)