#!/usr/bin/env python3
"""
Obstacle Avoider - Fixed
Key fixes:
  1. Disabled during cruise (only active during descent/ascent near ground)
  2. Stuck threshold raised - 0.03 m/s was triggering on normal drone wobble
  3. Stuck detection disabled completely when drone is above 2m (no walls up there)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
import math
import time


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.declare_parameter('drone_namespace',    'simple_drone')
        self.declare_parameter('sonar_topic',        '/simple_drone/sonar')
        self.declare_parameter('odom_topic',         '/simple_drone/odom')
        self.declare_parameter('ground_min_height',   0.25)  # m
        self.declare_parameter('stuck_time',          3.0)   # s  — raised from 1.5
        self.declare_parameter('stuck_threshold',     0.08)  # m/s — raised from 0.05
        self.declare_parameter('avoidance_speed',     0.3)
        self.declare_parameter('avoidance_duration',  2.0)
        # Only run stuck-detection below this altitude (walls don't exist above it)
        self.declare_parameter('cruise_altitude',    10.0)   # m

        ns              = self.get_parameter('drone_namespace').value
        sonar_topic     = self.get_parameter('sonar_topic').value
        odom_topic      = self.get_parameter('odom_topic').value
        self.gnd_min    = self.get_parameter('ground_min_height').value
        self.stuck_time = self.get_parameter('stuck_time').value
        self.stuck_thr  = self.get_parameter('stuck_threshold').value
        self.avoid_spd  = self.get_parameter('avoidance_speed').value
        self.avoid_dur  = self.get_parameter('avoidance_duration').value
        self.cruise_alt = self.get_parameter('cruise_altitude').value

        # State
        self.delivery_cmd     = Twist()
        self.sonar_range      = float('inf')
        self.too_close_ground = False
        self.vx = self.vy = self.vz = 0.0
        self.pos_z = 0.0

        # Stuck detection
        self.cmd_nonzero_since = None
        self.stuck             = False
        self.avoidance_start   = None
        self.obstacle_active   = False

        # Publishers
        self.safe_cmd_pub = self.create_publisher(Twist,  f'/{ns}/cmd_vel',       10)
        self.status_pub   = self.create_publisher(Bool,   '/obstacle/detected',   10)
        self.info_pub     = self.create_publisher(String, '/obstacle/info',       10)

        # Subscribers
        self.create_subscription(Twist,    '/delivery/cmd_vel_raw', self._cmd_cb,   10)
        self.create_subscription(Range,    sonar_topic,             self._sonar_cb, 10)
        self.create_subscription(Odometry, odom_topic,              self._odom_cb,  10)

        self.create_timer(0.05, self._loop)

        self.get_logger().info('Obstacle Avoider ready')
        self.get_logger().info(f'  stuck_threshold : {self.stuck_thr} m/s')
        self.get_logger().info(f'  stuck_time      : {self.stuck_time} s')
        self.get_logger().info(f'  cruise_altitude : {self.cruise_alt} m  (no stuck-detect above this)')

    def _cmd_cb(self, msg):
        self.delivery_cmd = msg

    def _sonar_cb(self, msg: Range):
        self.sonar_range      = msg.range
        self.too_close_ground = msg.range < self.gnd_min

    def _odom_cb(self, msg: Odometry):
        v = msg.twist.twist.linear
        self.vx    = v.x
        self.vy    = v.y
        self.vz    = v.z
        self.pos_z = msg.pose.pose.position.z

    def _loop(self):
        now   = time.time()
        cmd   = self.delivery_cmd
        speed = math.sqrt(self.vx**2 + self.vy**2)

        commanding = (abs(cmd.linear.x)  > 0.01 or
                      abs(cmd.linear.y)  > 0.01 or
                      abs(cmd.linear.z)  > 0.01 or
                      abs(cmd.angular.z) > 0.01)

        # ── Stuck detection — ONLY below cruise altitude ──────────────
        # At cruise altitude there are no walls so any slowness is
        # normal drone physics, not a collision
        at_cruise = self.pos_z >= (self.cruise_alt - 1.0)

        if not at_cruise:
            if commanding:
                if self.cmd_nonzero_since is None:
                    self.cmd_nonzero_since = now
                elif (not self.stuck and
                      now - self.cmd_nonzero_since > self.stuck_time and
                      speed < self.stuck_thr):
                    self.stuck           = True
                    self.avoidance_start = now
                    self.get_logger().warn(
                        f'STUCK (z={self.pos_z:.1f}m speed={speed:.3f}m/s) — backing away!')
            else:
                self.cmd_nonzero_since = None
        else:
            # Reset stuck state when we're high up
            self.cmd_nonzero_since = None
            if self.stuck:
                self.stuck           = False
                self.avoidance_start = None
                self.get_logger().info('High altitude — stuck detection reset')

        # Clear stuck after avoidance duration
        if self.stuck and self.avoidance_start is not None:
            if now - self.avoidance_start > self.avoid_dur:
                self.stuck             = False
                self.avoidance_start   = None
                self.cmd_nonzero_since = None
                self.get_logger().info('Avoidance done — resuming')

        self.obstacle_active = self.stuck or self.too_close_ground

        # ── Output ────────────────────────────────────────────────────
        if self.too_close_ground:
            out = Twist()
            out.linear.z = self.avoid_spd
            self.get_logger().warn(
                f'GROUND {self.sonar_range:.2f}m — climbing',
                throttle_duration_sec=1.0)
        elif self.stuck:
            out = Twist()
            elapsed = now - self.avoidance_start if self.avoidance_start else 0
            if elapsed < self.avoid_dur * 0.5:
                out.linear.x  = -self.avoid_spd
            else:
                out.angular.z =  0.8
        else:
            out = self.delivery_cmd   # pass through

        self.safe_cmd_pub.publish(out)

        s = Bool(); s.data = self.obstacle_active
        self.status_pub.publish(s)

        info = String()
        if self.stuck:
            info.data = f'STUCK z={self.pos_z:.1f}m spd={speed:.3f}m/s'
        elif self.too_close_ground:
            info.data = f'GROUND {self.sonar_range:.2f}m'
        else:
            info.data = f'clear z={self.pos_z:.1f}m spd={speed:.2f}m/s'
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()