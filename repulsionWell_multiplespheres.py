#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from copy import copy # To make shallow copy last vector distance

from math import sqrt # To calculate distance

import argparse       # To make easier input parameter parsing

# Default dynamic parameters used for the repulsion (see main)
K = 100.0
B = 0.01
NANOSEC = 1000000

# To round each element of a float list for display purposes
def roundlist(list,n):
    l_round = []
    for element in list:
        l_round.append(round(element,n))
    return l_round


# Node used to simulate a repulsion well effect around defined spheric markers of visualization_marker topic
# Force is calculated based on f = k*d+b*v, where d is vector from center of the well to current one and v is its associated speed
# Only repulsion forced calculated is from the nearest object (all others are ignored)
# Furthermore, verbose allows message on terminal to be printed (> 0)
class GravityWell(Node):
    def __init__(self,k,b,verbose):
        super().__init__('gravityWell')
        self._pub_wrench = self.create_publisher(WrenchStamped, '/arm/servo_cf', 5)
        self._sub_pose = self.create_subscription(PoseStamped, '/arm/measured_cp', self._pose_cb, 10)
        self._sub_markers = self.create_subscription(Marker, '/visualization_marker', self._markers_cb,10)

        # Set inner paramaters for computing (see cb)
        self.k = k
        self.b = b
        self.v = [0.0,0.0,0.0]
        self.lastd = None
        self.verbose = verbose

        self.datareceived = False
        self.positions = []
        self.r = 0.0

        if self.verbose > 0:
            self.get_logger().info(f'Initiating repulsion well forces with dynamic parameters k (N/m): {round(self.k,2)} and b (N*s/m): {round(self.b,5)}\n')


    def _pose_cb(self, msg: PoseStamped):

        # Waits 'til marker data is obtained
        if self.datareceived and len(self.positions) > 0:

            # Position (p), vector from position to well center (d) and its distance (dmod)
            p = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,msg.header.stamp.nanosec]

            f = [0.0,0.0,0.0] # Force to calculate sent to robot

            # Position and distance to the nearest object center
            nearest_d = None
            nearest_dmod = -1.0

            for position in self.positions:

                d = [position.x[0]-p[0],position.y-p[1],position.z-p[2],p[3]]
                dmod = sqrt(d[0]**2+d[1]**2+d[2]**2)

                if nearest_dmod > 0 and nearest_dmod > dmod:
                    nearest_dmod = dmod
                    nearest_d = d

            lapsed = 0 # Future time (on ns) passed between current callback and previous

            # Set a velocity based on vector d and previous d of prior callback (lastd)
            # Set lapsed before, using time stamp on nanosec to get sample period
            # (velocity will be set to 0 if lapsed is 0)
            if self.lastd is not None:
                lapsed = nearest_d[3]-self.lastd[3]
                if lapsed != 0:
                    for i in range(len(self.v)):
                        self.v[i] = (nearest_d[i]-self.lastd[i])/lapsed*NANOSEC
                elif self.verbose > 0:
                    self.get_logger().info('WARNING: Position updated without timestamp change, setting velocity to 0...')
            self.lastd = copy(nearest_d)

            # Set force to the repulsion force of the well (f = -(k*d+b*v)) if it is on the sphere and not in the center
            # (Force is in the opposite direction than d or v)
            if nearest_dmod < self.r and nearest_dmod > 0:
                for i in range(len(f)):
                    f[i] = -(self.k*nearest_d[i]+self.b*self.v[i])

            if self.verbose > 0:
                self.get_logger().info(f'Calculated force (N): {roundlist(f,4)}.\n\
                                         Distance (m) to center is {round(nearest_dmod,4)} from point {roundlist(p,4)} and current velocity (m/s) is {roundlist(self.v,4)}.\n\
                                         Time (ns) passed between this force update and previous: {lapsed}\n\n')

            # Prepare force value as wrench to be published
            cmd = WrenchStamped()
            cmd.wrench.force.x = f[0]
            cmd.wrench.force.y = f[1]
            cmd.wrench.force.z = f[2]

            self._pub_wrench.publish(cmd)

    # Stores marker data, then deactivates the subscription
    def _markers_cb(self, msg: Marker):

        if not self.datareceived:
            self.r = msg.scale.x/2

            for point in msg.points:
                self.positions.append(point)

            self.datareceived = True

    # Set force to 0 once gravity well is deactivated on an unexpected way (NOTE: Implement ctrl+c exit)
    def sentforce0(self):
        cmd = WrenchStamped()
        cmd.wrench.force.x = 0.0
        cmd.wrench.force.y = 0.0
        cmd.wrench.force.z = 0.0

        self._pub_wrench.publish(cmd)


def main():

    # Program jumps here directly as how it is defined on setup files
    parser = argparse.ArgumentParser(description='Node to create a spheric gravity well for a phantom robot (no other force publisher must be present)',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--wellK', '-wk', type=float, default = K, help = 'Dynamic parameter of the well pull (stiffness, N/m)')
    parser.add_argument('--wellB', '-wb', type=float, default = B, help = 'Dynamic parameter of the well pull (cushioning, N*s/m)')

    parser.add_argument('--verbose','-v', type = int, default = 0, help = 'Show information on terminal if > 0')

    args = parser.parse_args()


    rclpy.init()
    node = GravityWell(args.wellK,args.wellB,args.verbose)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # node.sentforce0()
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown()


if __name__ == '__main__':
    main()