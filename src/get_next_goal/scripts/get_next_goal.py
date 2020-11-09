#!/usr/bin/env python
import rospy
from gazebo_msgs2.srv import GetNextPoint, GetNextPointResponse, GetNextColor, GetNextColorResponse
from geometry_msgs.msg import Point
from std_msgs.msg import String
import random
from math import sin, cos, pi, floor

colors = ['red', 'blue', 'grey', 'white']

def generate_point(r):
    alpha = random.random() * 2 * pi
    return Point(r * cos(alpha), r * sin(alpha), 0)

def get_point(req):
    return GetNextPointResponse(generate_point(10))

def get_color(req):
    color = String(colors[floor(random.random() * len(colors))])
    return GetNextColorResponse(color)

def pub_sound():
    rospy.init_node('get_next_goal', anonymous=True)
    s1 = rospy.Service('get_point', GetNextPoint, get_point)
    s2 = rospy.Service('get_color', GetNextColor, get_color)
    rospy.spin()

if __name__ == '__main__':
    try:
        pub_sound()
    except rospy.ROSInterruptException:
        pass