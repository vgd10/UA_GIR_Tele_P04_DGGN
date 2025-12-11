#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import argparse       # To make easier input parameter parsing

# The position of the sphere was obtained experimentally
PWELL = [0.0127,0.0108,0.0855]
R = 0.05
POSITIONS = [[0.0,0.0,0.0],[0.5,0.5,0.5]]


class SpherePublisher(Node):
    def __init__(self,pwell,r,positions,verbose):
        super().__init__('sphere_publisher')

        # Publishing in the /visualization_marker topic
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        timer_period = 0.5 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pwell=pwell
        self.r=r
        self.positions = positions
        self.verbose = verbose

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = "stilus"      
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "basic_shapes"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        # Sphere position
        marker.pose.position.x = self.pwell[0]
        marker.pose.position.y = self.pwell[1]
        marker.pose.position.z = self.pwell[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Scale (for the diameter of the sphere)
        marker.scale.x = 2*self.r
        marker.scale.y = 2*self.r
        marker.scale.z = 2*self.r

        # Position of spheres
        for position in self.positions:
            point = Point()
            point.x = position[0]
            point.y = position[1]
            point.z = position[2]
            marker.points.add(point)

        # Color (RGBA)
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 1.0  # 1.0 = opaco

        # Persistence
        marker.lifetime.sec = rclpy.Duration()

        self.publisher.publish(marker)

        if self.verbose > 0:
            self.get_logger().info("Esfera publicada en RViz")


def main():

    # Program jumps here directly as how it is defined on setup files
    parser = argparse.ArgumentParser(description='Node to create a sphere)',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--wellCenter', '-wc', type=float, nargs=3, default = PWELL, help = 'Center of the well (m)')
    parser.add_argument('--wellRadius', '-wr', type=float, default = R, help = 'Radius of the sphere that defines the well (m)')

    parser.add_argument('--verbose','-v', type = int, default = 0, help = 'Show information on terminal if > 0')

    #args = parser.parse_args()
    args, unknown = parser.parse_known_args()

    positions = POSITIONS

    rclpy.init()
    node = SpherePublisher(args.wellCenter,args.wellRadius,positions,args.verbose)
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

