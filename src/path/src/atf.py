from geometry_msgs.msg import Point
from math import sqrt, sin, cos

origin = Point(x=0, y=0, z=0)

def inverse(p):
    return Point(x=-p.x, y=-p.y, z=0)

def transformBy(p, by):
    return Point(x=p.x+by.x, y=p.y+by.y, z=0)

def rotateBy(p, theta):
    return Point(x=(p.x*cos(theta) - p.y*sin(theta)), y=(p.x*sin(theta) + p.y*cos(theta)))

def distance(a,b):
    return sqrt( (a.x-b.x)**2 + (a.y - b.y) ** 2 )
