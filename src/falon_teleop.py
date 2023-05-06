#!/usr/bin/python3
import rclpy
import math
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from falcon_interfaces.msg import FalconPos,FalconForces

from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('lidar_detect')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile=qos_profile_sensor_data)
        self.subscription
        self.publisher = self.create_publisher(Twist, 'cmd_vel',10)
        self.pub_force = self.create_publisher(FalconForces, 'sendForces',10)
        self.sub_falcon = self.create_subscription(
            FalconPos,
            'readFalcon',
            self.control_callback,
            10)
        
        self.pos = [0.0,0.0,0.0]
        self.min_distance= 0.30
        self.m_stiffness = 1000
        self.velocity = 0.0
        self.angular = 0.0
        self.kv = -5.0
        self.ka = 9.09
        self.kf = -43.3
        self.init_pos_f = 0.13
        self.init_pos_b = 0.11
        self.force = [0.0,0.0,0.0]
        self.is_obstacle_close = False

    def control_callback(self,msg:FalconPos):
        self.pos[0] = msg.x
        self.pos[1] = msg.y
        self.pos[2] = msg.z
        cmd_vel = Twist()

        if self.pos[2] >= 0.11:
            self.velocity  = self.kv*(self.pos[2] - self.init_pos_f)
        elif self.pos[2] <= 0.13:
            self.velocity  = self.kv*(self.pos[2] - self.init_pos_b)
        elif self.pos[0] <= -0.03 :
            self.angular = 0.3
            self.get_logger().info("left")
        elif self.pos[0] >= 0.03:
            self.angular = -0.3
            self.get_logger().info("right")
        elif self.pos[1] <= -0.03:
            self.velocity  = 0.0
            self.angular = 0.0
            self.get_logger().info("break")
        else:
            self.velocity  = 0.0
            self.angular = 0.0
        cmd_vel.linear.x = self.velocity
        cmd_vel.angular.z = self.angular
        self.publisher.publish(cmd_vel)

    def listener_callback(self, msg:LaserScan):
        distance = msg.ranges[0]
        self.get_logger().info("distance:%s"% distance)
        force = FalconForces()
        if ((distance!=0.0) < self.min_distance):
            self.force = self.kf * (distance - self.min_distance)
            force.z = self.force
            self.pub_force.publish(force)
            self.get_logger().info("Close to the wall")
        else:
            self.is_obstacle_close = False
            force.z = 0.0
            self.pub_force.publish(force)
            self.get_logger().info("Not Close Obstacle, Let move")

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