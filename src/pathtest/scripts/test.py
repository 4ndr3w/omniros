from Segment import Segment
from Waypoint import Waypoint, distance

start = Waypoint(0, 2)
end = Waypoint(3, 3)

robot = Waypoint(1.6707, 2)

d1 = distance(robot, start)
d2 = distance(robot, end)


seg = Segment(start, end)

print(seg.distance)

print(d1+d2)
