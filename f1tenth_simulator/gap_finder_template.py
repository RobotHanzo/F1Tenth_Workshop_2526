#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class GapFinderStarter(Node):

    def __init__(self):
        super().__init__('gap_finder_starter')

        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('max_steering', 0.34)
        self.declare_parameter('bubble_size', 10)
        
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering = self.get_parameter('max_steering').value
        self.bubble_size = self.get_parameter('bubble_size').value

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        self.pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)

    # ----------------------------- 
    # Implement gap finding methods below

    def preprocess(self, ranges):
        ranges = np.array(ranges)
        ranges[np.isnan(ranges)] = 0
        ranges[np.isinf(ranges)] = 0
        return ranges

    def remove_closest_obstacle(self, ranges):
        closest = np.argmin(ranges[ranges > 0])
        
        start = max(0, closest - self.bubble_size)
        end = min(len(ranges)-1, closest + self.bubble_size)
        ranges[start:end] = 0
        return ranges

    def find_best_point(self, ranges):
        return np.argmax(ranges)

    # -----------------------------

    def lidar_callback(self, msg):

        ranges = self.preprocess(msg.ranges)
        ranges = self.remove_closest_obstacle(ranges)
        best_index = self.find_best_point(ranges)

        steering = msg.angle_min + best_index * msg.angle_increment
        steering = np.clip(steering, -self.max_steering, self.max_steering)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering
        drive_msg.drive.speed = self.max_speed

        self.pub.publish(drive_msg)


def main():
    rclpy.init()
    node = GapFinderStarter()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
