#!/usr/bin/env python3
"""
coordinate_mission_controller.py
Place at: src/drone_delivery_system/drone_delivery_system/coordinate_mission_controller.py
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, String, Int8, Bool
from std_srvs.srv import Empty as EmptySrv

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
from gazipin import decode as gazipin_decode, validate as gazipin_validate
from gazipin import CELL_SIZE_M

# ── drone plugin navi_state constants (matches plugin_drone_private.h) ──
LANDED_MODEL    = 0
TAKINGOFF_MODEL = 1
FLYING_MODEL    = 2
LANDING_MODEL   = 3

# ── mission states ───────────────────────────────────────────────────
IDLE      = 'idle'
TAKEOFF   = 'takeoff'   # waiting for drone to reach FLYING_MODEL
CLIMB     = 'climb'
GOTO      = 'goto'
DESCEND   = 'descend'
LAND      = 'land'

KNOWN = {
    'K22-222-22': 'Origin           ( 0.00,  0.00) m',
    'K2J-F64-3M': 'Person Standing  ( 4.40,  2.40) m',
    'K2K-97F-PM': 'Dumpster         ( 3.71,  4.45) m',
    'J54-95C-6C': 'Fire Hydrant     ( 0.45, -1.66) m',
    'J57-K47-PC': 'Cardboard Boxes  ( 2.39, -3.68) m',
    '8CT-PP5-CM': 'Table            (-6.33,  5.25) m',
    'K22-772-7T': 'Test point       ( 0.50,  0.50) m',
    'K22-222-7K': 'Test point       ( 0.01,  0.01) m',
    'K2J-M93-5P': 'Near Person S    ( 4.40,  1.90) m  [0.5m south]',
    'K2J-T74-5M': 'Near Person N    ( 4.40,  2.90) m  [0.5m north]',
    'K2P-4C4-JM': 'Near Person E    ( 4.90,  2.40) m  [0.5m east]',
    'K2J-8MF-JM': 'Near Person W    ( 3.90,  2.40) m  [0.5m west]',
}


def _prompt() -> tuple:
    print()
    print('╔══════════════════════════════════════════════════════════════╗')
    print('║         ROS2 Drone — GAZIPIN Mission Controller             ║')
    print('╠══════════════════════════════════════════════════════════════╣')
    print(f'║  Cell size : {CELL_SIZE_M*100:.4f} cm  │  World : X/Y ∈ [-50, 50] m    ║')
    print('╠══════════════════════════════════════════════════════════════╣')
    print('║  Known locations (from home.sdf):                           ║')
    for code, label in KNOWN.items():
        print(f'║    {code}  →  {label:<33}║')
    print('╠══════════════════════════════════════════════════════════════╣')
    print('║  Format: XXX-XXX-XX   e.g.  K2J-M93-5P  →  0.5m from person║')
    print('╚══════════════════════════════════════════════════════════════╝')
    print()
    while True:
        try:
            raw = input('  Enter GAZIPIN > ').strip()
            if not raw:
                continue
            ok, err = gazipin_validate(raw)
            if not ok:
                print(f'  ✗  {err}')
                continue
            x, y = gazipin_decode(raw)
            hint = KNOWN.get(raw.upper().replace('-', ''), '')
            note = f'  ({hint.split("(")[0].strip()})' if hint else ''
            print(f'  ✓  ({x:.4f}, {y:.4f}) m{note}')
            return x, y
        except (EOFError, KeyboardInterrupt):
            print('\n  Aborted.')
            sys.exit(0)


class CoordinateMissionController(Node):

    def __init__(self, target_x: float, target_y: float):
        super().__init__('coordinate_mission_controller')

        # ── CRITICAL: use simulation time, not wall clock ────────────
        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True
            )
        ])

        self.declare_parameter('drone_namespace', 'simple_drone')
        self.declare_parameter('cruise_alt',  3.0)
        self.declare_parameter('stop_radius', 0.25)
        self.declare_parameter('land_sonar',  0.5)
        self.declare_parameter('sonar_topic', '/simple_drone/sonar')
        self.declare_parameter('odom_topic',  '/simple_drone/odom')

        ns               = self.get_parameter('drone_namespace').value
        self.cruise_alt  = self.get_parameter('cruise_alt').value
        self.stop_radius = self.get_parameter('stop_radius').value
        self.land_sonar  = self.get_parameter('land_sonar').value

        self.target_x = target_x
        self.target_y = target_y

        # runtime
        self.state       = IDLE
        self.px = self.py = self.pz = 0.0
        self.yaw         = 0.0
        self.sonar       = 99.0
        self.land_t      = None
        self.carrying    = True
        self.vz              = 0.0
        self.descent_dropping = False
        self.drone_state = LANDED_MODEL   # from /simple_drone/state topic
        self.takeoff_sent_t = None

        # publishers
        self.cmd_pub     = self.create_publisher(Twist,  f'/{ns}/cmd_vel',  10)
        self.takeoff_pub = self.create_publisher(Empty,  f'/{ns}/takeoff',  10)
        self.land_pub    = self.create_publisher(Empty,  f'/{ns}/land',     10)
        self.posctrl_pub = self.create_publisher(Bool,   f'/{ns}/posctrl',  10)
        self.state_pub   = self.create_publisher(String, '/delivery/state', 10)
        self.drop_cli    = self.create_client(EmptySrv, '/payload/drop')

        # subscribers
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 10)
        self.create_subscription(
            Range, self.get_parameter('sonar_topic').value, self._sonar_cb, 10)
        # listen to the drone plugin's own state (LANDED/TAKINGOFF/FLYING/LANDING)
        self.create_subscription(
            Int8, f'/{ns}/state', self._drone_state_cb, 10)

        self.create_service(EmptySrv, '/delivery/start', self._start_cb)
        self.create_timer(0.1, self._loop)

        self.get_logger().info(
            f'Target ({self.target_x:.4f}, {self.target_y:.4f}) m  |  '
            f'cruise={self.cruise_alt}m  stop={self.stop_radius}m  '
            f'land_sonar={self.land_sonar}m')
        self.get_logger().info('Auto-starting in 2 s …')
        self._start_timer = self.create_timer(2.0, self._auto_start)

    # ── start ────────────────────────────────────────────────────────

    def _auto_start(self):
        if self.state != IDLE:
            return
        self._start_timer.cancel()
        self.get_logger().info('═══ MISSION START — sending takeoff ═══')

        # ensure velocity mode (posctrl = False)
        b = Bool(); b.data = False
        self.posctrl_pub.publish(b)

        self.takeoff_pub.publish(Empty())
        self.takeoff_sent_t = time.time()
        self._go(TAKEOFF)

    def _start_cb(self, _req, res):
        self._auto_start()
        return res

    # ── sensors ──────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.px, self.py, self.pz = p.x, p.y, p.z
        self.vz = msg.twist.twist.linear.z   # vertical velocity
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

    def _sonar_cb(self, msg: Range):
        self.sonar = msg.range

    def _drone_state_cb(self, msg: Int8):
        self.drone_state = msg.data

    # ── control loop ─────────────────────────────────────────────────

    def _loop(self):
        {TAKEOFF: self._do_takeoff,
         CLIMB:   self._do_climb,
         GOTO:    self._do_goto,
         DESCEND: self._do_descend,
         LAND:    self._do_land,
         }.get(self.state, lambda: None)()

    # ── TAKEOFF: wait until plugin reports FLYING_MODEL ──────────────

    def _do_takeoff(self):
        self.get_logger().info(
            f'TAKEOFF  waiting for FLYING state '
            f'(drone_state={self.drone_state})',
            throttle_duration_sec=1.0)

        if self.drone_state == FLYING_MODEL:
            self.get_logger().info('Drone is FLYING → CLIMB')
            self._go(CLIMB)
            return

        # safety: if stuck for > 5s resend takeoff
        if self.takeoff_sent_t and time.time() - self.takeoff_sent_t > 5.0:
            self.get_logger().warn('Resending takeoff …')
            self.takeoff_pub.publish(Empty())
            self.takeoff_sent_t = time.time()

    # ── CLIMB: rise to cruise altitude ───────────────────────────────

    def _do_climb(self):
        err = self.cruise_alt - self.pz
        self.get_logger().info(
            f'CLIMB  z={self.pz:.2f}/{self.cruise_alt:.1f} m  (payload on)',
            throttle_duration_sec=1.0)

        if err < 0.10:
            # stop all motion before transitioning — kills upward momentum
            self._stop()
            # brief hover: keep sending zero until altitude stabilises
            if abs(err) < 0.15:
                self._go(GOTO)
            return

        t = Twist()
        # slow down as we approach — prevents overshoot
        t.linear.z = min(0.4, max(0.05, err * 0.4))
        self.cmd_pub.publish(t)

    # ── GOTO: fly at cruise altitude, stop 0.25 m short ──────────────

    def _do_goto(self):
        dx   = self.target_x - self.px
        dy   = self.target_y - self.py
        dist = math.sqrt(dx * dx + dy * dy)

        if dist <= self.stop_radius:
            self.get_logger().info(
                f'GOTO  reached ({dist:.3f} m from target) → DESCEND')
            self._stop()
            self.descent_dropping = False   # reset for fresh descent
            self._go(DESCEND); return

        target_yaw = math.atan2(dy, dx)
        yaw_err    = target_yaw - self.yaw
        if yaw_err >  math.pi: yaw_err -= 2 * math.pi
        if yaw_err < -math.pi: yaw_err += 2 * math.pi

        self.get_logger().info(
            f'GOTO  dist={dist:.3f} m  Δyaw={math.degrees(yaw_err):+.1f}°  '
            f'z={self.pz:.2f} m',
            throttle_duration_sec=1.0)

        t = Twist()
        t.linear.x  = min(0.6, (dist - self.stop_radius) * 0.4)
        t.angular.z = yaw_err * 1.5
        # strong symmetric hold — prevents upward drift during horizontal flight
        alt_err     = self.cruise_alt - self.pz
        t.linear.z  = max(-0.5, min(0.5, alt_err * 1.5))
        self.cmd_pub.publish(t)

    # ── DESCEND: hover to kill momentum, then land command + monitor pz ──
    #
    # The plugin clamps thrust to >= 0 (force[2] = 0 if negative).
    # Sending any negative linear.z just means zero thrust — gravity takes over.
    # But the drone has upward momentum from CLIMB so it drifts up first.
    #
    # Fix:
    #   Phase 1 (BLEED): send linear.z = 0 (hover) until vertical velocity
    #                    is near zero — bleeds the upward momentum.
    #   Phase 2 (DROP):  publish /land (cuts thrust to 80% for 1s then 0)
    #                    and watch pz via odom. Once pz < 0.3m → done.

    def _do_descend(self):
        self.get_logger().info(
            f'DESCEND  z={self.pz:.3f} m  vz={self.vz:+.3f} m/s  sonar={self.sonar:.3f} m',
            throttle_duration_sec=0.5)

        # sonar trusted only below 1m (above that reads walls)
        if self.pz <= 1.0 and self.sonar <= self.land_sonar:
            self.get_logger().info(f'sonar={self.sonar:.3f} m → LAND')
            self._stop(); self._go(LAND); return

        # The plugin clamps force[2] >= 0, so any negative linear.z = zero thrust.
        # Strategy: send -2.0 to force zero thrust so gravity pulls drone down.
        # But limit fall speed to 0.5 m/s by briefly hovering when too fast.
        if self.vz < -0.5:
            # falling too fast — apply brief hover to slow down
            t = Twist(); t.linear.z = 1.0
            self.cmd_pub.publish(t)
        else:
            # send large negative to zero out thrust → gravity descends drone
            t = Twist(); t.linear.z = -2.0
            self.cmd_pub.publish(t)


    # ── LAND ─────────────────────────────────────────────────────────

    def _do_land(self):
        self._stop()
        if self.land_t is None:
            self.get_logger().info('🛬  LAND')
            self.land_pub.publish(Empty())
            self.land_t = time.time()
            # auto-call /payload/drop after 2 s (gives drone time to settle)
            self.create_timer(2.0, self._auto_drop)
            return
        if time.time() - self.land_t >= 7.0:
            self.get_logger().info('═══ MISSION COMPLETE ═══')
            self._go(IDLE)

    def _auto_drop(self):
        if not self.carrying:
            return
        self.get_logger().info('📦 Auto-dropping payload …')
        if self.drop_cli.service_is_ready():
            self.drop_cli.call_async(EmptySrv.Request())
        else:
            self.get_logger().warn(
                'payload_manager not running — start it and call '
                'ros2 service call /payload/drop std_srvs/srv/Empty manually')
        self.carrying = False

    # ── helpers ───────────────────────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _go(self, s: str):
        self.state = s
        m = String(); m.data = s
        self.state_pub.publish(m)
        self.get_logger().info(f'──── {s.upper()} ────')


def main(args=None):
    tx, ty = _prompt()
    print(f'  Starting ROS2 node — target=({tx:.4f}, {ty:.4f}) m')
    rclpy.init(args=args)
    node = CoordinateMissionController(tx, ty)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()