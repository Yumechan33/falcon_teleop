#!/usr/bin/python3
import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from falcon_interfaces.msg import FalconPos,FalconForces

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('lidar_detect')
        # self.subscription = self.create_subscription(
        #     LaserScan,
        #     'scan',
        #     self.listener_callback,
        #     10)
        # self.subscription
        self.publisher = self.create_publisher(Twist, 'cmd_vel',10)
        # self.pub_force = self.create_publisher(FalconForces, 'sendForces',10)
        self.sub_falcon = self.create_subscription(
            FalconPos,
            'readFalcon',
            self.control_callback,
            10)
        self.goal_tetra = 0.0
        self.pos = [0,0,0]
        self.min_distance= 0.5
        self.m_stiffness = 1000
        self.velocity = 0
        self.angular = 0
        self.k = 0.5
        self.gain = 4
        self.init_pos = 0.07
        self.force = [0.0,0.0,0.0]
        self.is_obstacle_close = False

    def control_callback(self,msg:FalconPos):
        self.pos[0] = msg.x
        self.pos[1] = msg.y
        self.pos[2] = msg.z
        cmd_vel = Twist()
        force = FalconForces()

        if self.pos[2] >= 0.08:
                cmd_vel.linear.x = self.k*(self.pos[2] - self.init_pos)* self.gain
                if cmd_vel.linear.x >= 0.22:
                      cmd_vel.linear.x = 0.22

                cmd_vel.angular.z = 0.0
        # elif self.pos[1] <= -0.03:
        #         cmd_vel.linear.x = -0.5
        #         cmd_vel.angular.z = 0.0
        elif self.pos[0] <= -0.03:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.5
        elif self.pos[0] >= 0.03:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = -0.5
        else:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        self.velocity = cmd_vel.linear.x
        self.angular = cmd_vel.angular.z
        self.publisher.publish(cmd_vel)
        


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()