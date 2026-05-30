#!/usr/bin/env python3
"""
Delivery Controller
===================
Mission:
  TAKEOFF
    → CLIMB      : rise to 2 m
    → SPIN       : rotate with front camera until person found
    → GOTO_PERSON: fly toward person (front cam + odometry waypoint)
    → WAIT_BOTTOM: hover and slowly creep until bottom camera sees person
                   — this means we are directly above them
    → LAND_DESC  : descend slowly to sonar = 0.05 m (no detection needed)
    → LAND       : send land command

Start: ros2 service call /delivery/start std_srvs/srv/Empty
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Empty, String
from std_srvs.srv import Empty as EmptySrv
from vision_msgs.msg import Detection2DArray

IDLE        = 'idle'
CLIMB       = 'climb'
SPIN        = 'spin'
GOTO_PERSON = 'goto_person'
WAIT_BOTTOM = 'wait_bottom'
LAND_DESC   = 'land_descend'
LAND        = 'land'

IMG_CX           = 320.0
IMG_CY           = 240.0
CLOSE_BOX_WIDTH  = 40      # px  — front cam: person wide enough to switch to WAIT_BOTTOM
HFOV             = 2.09    # rad — front camera horizontal FOV
PERSON_DIST_INIT = 8.0     # m   — assumed initial distance for world pos estimate


class DeliveryController(Node):

    def __init__(self):
        super().__init__('delivery_controller')

        self.declare_parameter('drone_namespace', 'simple_drone')
        self.declare_parameter('fly_alt',         2.0)
        self.declare_parameter('land_sonar',      0.05)
        self.declare_parameter('sonar_topic',     '/simple_drone/sonar')
        self.declare_parameter('odom_topic',      '/simple_drone/odom')

        ns              = self.get_parameter('drone_namespace').value
        self.fly_alt    = self.get_parameter('fly_alt').value
        self.land_sonar = self.get_parameter('land_sonar').value

        # runtime
        self.state      = IDLE
        self.px = self.py = self.pz = 0.0
        self.yaw        = 0.0
        self.sonar      = 99.0

        self.front_dets    = None
        self.bottom_dets   = None
        self.last_person_t = None
        self.last_err_x    = 0.0

        # odometry-based person position estimate
        self.person_wx  = None
        self.person_wy  = None
        self.person_set = False

        self.spin_accum = 0.0
        self.last_yaw   = None
        self.land_t     = None

        # publishers
        self.cmd_pub     = self.create_publisher(Twist,  f'/{ns}/cmd_vel',  10)
        self.takeoff_pub = self.create_publisher(Empty,  f'/{ns}/takeoff',  10)
        self.land_pub    = self.create_publisher(Empty,  f'/{ns}/land',     10)
        self.state_pub   = self.create_publisher(String, '/delivery/state', 10)

        # subscribers
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,  self._odom_cb,       10)
        self.create_subscription(
            Range,    self.get_parameter('sonar_topic').value, self._sonar_cb,      10)
        self.create_subscription(
            Detection2DArray, '/detections/front',  self._front_det_cb,  10)
        self.create_subscription(
            Detection2DArray, '/detections/bottom', self._bottom_det_cb, 10)

        self.create_service(EmptySrv, '/delivery/start', self._start_cb)
        self.create_timer(0.1, self._loop)

        self.get_logger().info(
            f'Ready  fly={self.fly_alt}m  land_sonar={self.land_sonar}m')
        self.get_logger().info(
            'Start: ros2 service call /delivery/start std_srvs/srv/Empty')

    # ── callbacks ─────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.px, self.py, self.pz = p.x, p.y, p.z
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

    def _sonar_cb(self, msg: Range):
        self.sonar = msg.range

    def _front_det_cb(self, msg: Detection2DArray):
        persons = [
            d for d in msg.detections
            if d.results
            and d.results[0].hypothesis.class_id.lower() == 'person'
            and d.results[0].hypothesis.score >= 0.4
        ]
        if persons:
            self.front_dets    = persons
            self.last_person_t = time.time()
            det = max(persons, key=lambda d: d.bbox.size_x * d.bbox.size_y)
            self.last_err_x = det.bbox.center.position.x - IMG_CX
            # update world position estimate from bearing
            angle           = self.yaw + (self.last_err_x / IMG_CX) * (HFOV / 2.0)
            self.person_wx  = self.px + PERSON_DIST_INIT * math.cos(angle)
            self.person_wy  = self.py + PERSON_DIST_INIT * math.sin(angle)
            self.person_set = True
        else:
            self.front_dets = None

    def _bottom_det_cb(self, msg: Detection2DArray):
        persons = [
            d for d in msg.detections
            if d.results
            and d.results[0].hypothesis.class_id.lower() == 'person'
            and d.results[0].hypothesis.score >= 0.4
        ]
        self.bottom_dets = persons if persons else None

    # ── start ─────────────────────────────────────────────────────────

    def _start_cb(self, _req, res):
        if self.state != IDLE:
            self.get_logger().warn('Already running')
            return res
        self.get_logger().info('═══ MISSION START ═══')
        self.takeoff_pub.publish(Empty())
        self._go(CLIMB)
        return res

    # ── loop ──────────────────────────────────────────────────────────

    def _loop(self):
        {
            CLIMB:       self._do_climb,
            SPIN:        self._do_spin,
            GOTO_PERSON: self._do_goto_person,
            WAIT_BOTTOM: self._do_wait_bottom,
            LAND_DESC:   self._do_land_desc,
            LAND:        self._do_land,
        }.get(self.state, lambda: None)()

    # ── CLIMB ─────────────────────────────────────────────────────────

    def _do_climb(self):
        err = self.fly_alt - self.pz
        self.get_logger().info(
            f'CLIMB  z={self.pz:.2f} → {self.fly_alt}m',
            throttle_duration_sec=1.0)
        if err < 0.2:
            self._stop()
            self.spin_accum = 0.0
            self.last_yaw   = self.yaw
            self.person_set = False
            self._go(SPIN)
            return
        t = Twist()
        t.linear.z = min(0.5, err * 0.6)
        self.cmd_pub.publish(t)

    # ── SPIN ──────────────────────────────────────────────────────────

    def _do_spin(self):
        if self.last_yaw is not None:
            delta = self.yaw - self.last_yaw
            if delta >  math.pi: delta -= 2 * math.pi
            if delta < -math.pi: delta += 2 * math.pi
            self.spin_accum += abs(delta)
        self.last_yaw = self.yaw

        self.get_logger().info(
            f'SPIN  {math.degrees(self.spin_accum):.0f}°  '
            f'person={self.front_dets is not None}',
            throttle_duration_sec=1.0)

        if self.front_dets:
            self.get_logger().info('Person found! → GOTO_PERSON')
            self._stop()
            self._go(GOTO_PERSON)
            return

        if self.spin_accum >= 2 * math.pi:
            self.spin_accum = 0.0

        t = Twist()
        t.angular.z = 0.4
        t.linear.z  = (self.fly_alt - self.pz) * 0.4
        self.cmd_pub.publish(t)

    # ── GOTO_PERSON ───────────────────────────────────────────────────

    def _do_goto_person(self):
        """
        Fly toward person by odometry waypoint.
        When front cam bbox is wide enough OR odometry says close enough
        → switch to WAIT_BOTTOM to confirm we are above the person.
        """
        # if bottom camera already sees person we are above — skip ahead
        if self.bottom_dets:
            self.get_logger().info('Bottom cam sees person already → WAIT_BOTTOM')
            self._stop()
            self._go(WAIT_BOTTOM)
            return

        if not self.person_set:
            self.get_logger().warn('No position estimate → SPIN')
            self.spin_accum = 0.0
            self.last_yaw   = self.yaw
            self._go(SPIN)
            return

        dx   = self.person_wx - self.px
        dy   = self.person_wy - self.py
        dist = math.sqrt(dx * dx + dy * dy)

        target_yaw = math.atan2(dy, dx)
        yaw_err    = target_yaw - self.yaw
        if yaw_err >  math.pi: yaw_err -= 2 * math.pi
        if yaw_err < -math.pi: yaw_err += 2 * math.pi

        box_w = 0.0
        if self.front_dets:
            det   = max(self.front_dets,
                        key=lambda d: d.bbox.size_x * d.bbox.size_y)
            box_w = det.bbox.size_x

        self.get_logger().info(
            f'GOTO_PERSON  dist={dist:.1f}m  box_w={box_w:.0f}px',
            throttle_duration_sec=1.0)

        # close enough → go to WAIT_BOTTOM
        if box_w >= CLOSE_BOX_WIDTH or dist < 2.0:
            self.get_logger().info(
                f'Close (bw={box_w:.0f}px  dist={dist:.1f}m) → WAIT_BOTTOM')
            self._stop()
            self._go(WAIT_BOTTOM)
            return

        t = Twist()
        t.linear.x  = min(0.5, dist * 0.3)
        t.angular.z = yaw_err * 1.2
        t.linear.z  = (self.fly_alt - self.pz) * 0.4
        self.cmd_pub.publish(t)

    # ── WAIT_BOTTOM ───────────────────────────────────────────────────

    def _do_wait_bottom(self):
        """
        Hover and creep slowly using bottom camera to centre over person.
        The moment bottom camera sees the person AND they are centred
        (within tolerance) → start descending.
        If bottom camera doesn't see them yet, inch forward slowly.
        """
        if self.bottom_dets:
            det = max(self.bottom_dets,
                      key=lambda d: d.bbox.size_x * d.bbox.size_y)
            ex = det.bbox.center.position.x - IMG_CX   # left/right error
            ey = det.bbox.center.position.y - IMG_CY   # forward/back error

            self.get_logger().info(
                f'WAIT_BOTTOM  ex={ex:+.0f}px  ey={ey:+.0f}px',
                throttle_duration_sec=0.5)

            # centred enough → start descent
            if abs(ex) < 60 and abs(ey) < 60:
                self.get_logger().info(
                    'Centred over person → LAND_DESC')
                self._stop()
                self._go(LAND_DESC)
                return

            # nudge to centre
            t = Twist()
            t.linear.y = -ex * 0.0004   # left/right
            t.linear.x =  ey * 0.0004   # forward/back
            t.linear.z = (self.fly_alt - self.pz) * 0.3
            self.cmd_pub.publish(t)

        else:
            # bottom cam doesn't see person yet — inch forward slowly
            self.get_logger().info(
                'WAIT_BOTTOM  no bottom detection — inching forward',
                throttle_duration_sec=1.0)
            t = Twist()
            t.linear.x = 0.1
            self.cmd_pub.publish(t)

    # ── LAND_DESC ─────────────────────────────────────────────────────

    def _do_land_desc(self):
        """
        Descend slowly to sonar = 0.05 m.
        Keep centred with bottom camera if still visible.
        No abort, no SPIN — just go straight down.
        """
        self.get_logger().info(
            f'LAND_DESC  sonar={self.sonar:.3f}m → {self.land_sonar}m',
            throttle_duration_sec=0.5)

        if self.sonar <= self.land_sonar:
            self._stop()
            self._go(LAND)
            return

        t = Twist()
        t.linear.z = -0.08   # slow descent 8 cm/s

        if self.bottom_dets:
            det = max(self.bottom_dets,
                      key=lambda d: d.bbox.size_x * d.bbox.size_y)
            ex = det.bbox.center.position.x - IMG_CX
            ey = det.bbox.center.position.y - IMG_CY
            t.linear.y = -ex * 0.0003
            t.linear.x =  ey * 0.0003

        self.cmd_pub.publish(t)

    # ── LAND ──────────────────────────────────────────────────────────

    def _do_land(self):
        self._stop()
        if self.land_t is None:
            self.get_logger().info('🛬 LAND')
            self.land_pub.publish(Empty())
            self.land_t = time.time()
            return
        if time.time() - self.land_t >= 5.0:
            self.get_logger().info('═══ MISSION COMPLETE ═══')
            self._go(IDLE)

    # ── helpers ───────────────────────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _go(self, s):
        self.state = s
        m = String(); m.data = s
        self.state_pub.publish(m)
        self.get_logger().info(f'──── {s.upper()} ────')


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()