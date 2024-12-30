#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs2.srv import GetNextPoint, GetNextColor
from geometry_msgs.msg import Point
from std_msgs.msg import String
import random
from math import sin, cos, pi, floor

colors = ['red', 'blue', 'gren']

class GoalService(Node):

    def __init__(self):
        super().__init__('goal_service')
        self.srv1 = self.create_service(GetNextPoint, 'get_next_point', self.get_point_callback)
        self.srv2 = self.create_service(GetNextColor, 'get_next_color', self.get_color_callback)


    def get_point_callback(self, request, response):
        response.next_point = generate_point(10)
        #self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response

    def generate_point(r):
        alpha = random.random() * 2 * pi
        return Point(r * cos(alpha), r * sin(alpha), 0)
    
    def get_color_callback(self, request, response):
        response.color = String(colors[floor(random.random() * len(colors))])
        #self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():
    rclpy.init()

    goal_service = GoalService()

    rclpy.spin(goal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()