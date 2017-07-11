#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist, Vector3
import pygame

pygame.display.init()
pygame.joystick.init()


rospy.init_node('teleop')
rate = rospy.Rate(50)

js = pygame.joystick.Joystick(0)
js.init()

while not js.get_init():
    print("waiting for js init")

if not js:
    print("No joystick found")
    exit()

pub = rospy.Publisher('cmd_velocity', Twist, queue_size=10)

while not rospy.is_shutdown():
    vel = js.get_axis(1)
    omega = js.get_axis(3)
    
    vel *= -0.4
    omega *= -3.14

    linear = Vector3(0, vel, 0)
    angular = Vector3(0, 0, omega)
    msg = Twist(linear=linear, angular=angular)

    pub.publish(msg)

    pygame.event.pump()
    rate.sleep()