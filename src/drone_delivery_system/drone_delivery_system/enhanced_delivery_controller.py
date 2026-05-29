#!/usr/bin/env python3
"""
Simple Delivery Controller
Mission: takeoff → fly to cube → land on cube → wait for attach → ascend → 
         spin 360 find person → fly to person → descend → drop → land
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty as EmptySrv
import math
import time

IDLE       = 'idle'
CLIMB      = 'climb'
GOTO_CUBE  = 'goto_cube'
DESCEND    = 'descend'
WAIT       = 'wait_attach'
ASCEND     = 'ascend'
SPIN       = 'spin'
GOTO_PERSON= 'goto_person'
ALIGN      = 'align'
DROP_DESC  = 'drop_descend'
DROP       = 'drop'
LAND       = 'land'


class Controller(Node):
    def __init__(self):
        super().__init__('delivery_controller')

        self.declare_parameter('drone_namespace', 'simple_drone')
        self.declare_parameter('cube_x',          2.0)
        self.declare_parameter('cube_y',          1.5)
        self.declare_parameter('fly_alt',         3.0)
        self.declare_parameter('search_alt',      2.0)
        self.declare_parameter('drop_alt',        0.5)
        self.declare_parameter('sonar_topic',     '/simple_drone/sonar')
        self.declare_parameter('odom_topic',      '/simple_drone/odom')

        ns            = self.get_parameter('drone_namespace').value
        self.cube_x   = self.get_parameter('cube_x').value
        self.cube_y   = self.get_parameter('cube_y').value
        self.fly_alt  = self.get_parameter('fly_alt').value
        self.srch_alt = self.get_parameter('search_alt').value
        self.drop_alt = self.get_parameter('drop_alt').value

        # state
        self.state     = IDLE
        self.px = self.py = self.pz = 0.0
        self.yaw       = 0.0
        self.sonar     = 99.0
        self.front     = None
        self.bottom    = None
        self.attached  = False
        self.attach_t  = None
        self.drop_t    = None
        self.land_t    = None
        self.spin_accum = 0.0
        self.last_yaw  = None

        # pubs
        self.cmd   = self.create_publisher(Twist,  f'/{ns}/cmd_vel',  10)
        self.tkoff = self.create_publisher(Empty,  f'/{ns}/takeoff',  10)
        self.lnd   = self.create_publisher(Empty,  f'/{ns}/land',     10)
        self.stpub = self.create_publisher(String, '/delivery/state', 10)
        self.detach= self.create_publisher(Empty,  '/cube/detach',    10)

        # subs
        self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self._odom, 10)
        self.create_subscription(Range,    self.get_parameter('sonar_topic').value, self._snr,  10)
        self.create_subscription(Detection2DArray, '/detections/front',  self._fdet, 10)
        self.create_subscription(Detection2DArray, '/detections/bottom', self._bdet, 10)
        self.create_subscription(Empty,  '/cube/attach', self._on_attach, 10)
        self.create_subscription(String, '/cube/state',  self._on_state,  10)

        # start service
        self.create_service(EmptySrv, '/delivery/start', self._start)

        # detach at startup (plugin starts attached by default)
        self.create_timer(2.0, self._startup_detach)

        # main loop 10Hz
        self.create_timer(0.1, self._loop)

        self.get_logger().info(f'Ready. Cube=({self.cube_x},{self.cube_y}) fly={self.fly_alt}m search={self.srch_alt}m drop={self.drop_alt}m')
        self.get_logger().info(f'Start: ros2 service call /delivery/start std_srvs/srv/Empty')

    # ── callbacks ─────────────────────────────────────────────────────

    def _odom(self, m):
        self.px = m.pose.pose.position.x
        self.py = m.pose.pose.position.y
        self.pz = m.pose.pose.position.z
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def _snr(self, m):
        self.sonar = m.range

    def _fdet(self, m):
        ok = [d for d in m.detections if d.results and d.results[0].hypothesis.score > 0.4
              and d.results[0].hypothesis.class_id.lower() == 'person']
        self.front = ok if ok else None

    def _bdet(self, m):
        ok = [d for d in m.detections if d.results and d.results[0].hypothesis.score > 0.4
              and d.results[0].hypothesis.class_id.lower() == 'person']
        self.bottom = ok if ok else None

    def _on_attach(self, m):
        """User published /cube/attach — we hear it too"""
        if self.state == WAIT:
            self.attached  = True
            self.attach_t  = time.time()
            self.get_logger().info('✅ ATTACH received! Will ascend in 2.5s...')

    def _on_state(self, m):
        self.get_logger().info(f'[gz] cube: {m.data}')
        if 'attached' in m.data.lower() and self.state == WAIT:
            self.attached = True
            self.attach_t = self.attach_t or time.time()
            self.get_logger().info('✅ Plugin confirmed attach!')

    def _startup_detach(self):
        self.get_logger().info('Startup detach sent')
        self.detach.publish(Empty())

    def _start(self, req, res):
        if self.state != IDLE:
            return res
        self.get_logger().info('=== MISSION START ===')
        self.tkoff.publish(Empty())
        self._go(CLIMB)
        return res

    # ── loop ──────────────────────────────────────────────────────────

    def _loop(self):
        s = self.state
        if   s == IDLE:        return
        elif s == CLIMB:       self._do_climb()
        elif s == GOTO_CUBE:   self._do_goto_cube()
        elif s == DESCEND:     self._do_descend()
        elif s == WAIT:        self._do_wait()
        elif s == ASCEND:      self._do_ascend()
        elif s == SPIN:        self._do_spin()
        elif s == GOTO_PERSON: self._do_goto_person()
        elif s == ALIGN:       self._do_align()
        elif s == DROP_DESC:   self._do_drop_desc()
        elif s == DROP:        self._do_drop()
        elif s == LAND:        self._do_land()

    # ── CLIMB to fly_alt ──────────────────────────────────────────────

    def _do_climb(self):
        e = self.fly_alt - self.pz
        self.get_logger().info(f'CLIMB z={self.pz:.1f} → {self.fly_alt}m', throttle_duration_sec=1.0)
        if e < 0.3:
            self._stop(); self._go(GOTO_CUBE); return
        t = Twist(); t.linear.z = min(0.3, e*0.5)
        self.cmd.publish(t)

    # ── FLY to cube XY ────────────────────────────────────────────────

    def _do_goto_cube(self):
        dx = self.cube_x - self.px
        dy = self.cube_y - self.py
        d  = math.sqrt(dx*dx + dy*dy)
        ae = self.fly_alt - self.pz
        self.get_logger().info(f'GOTO_CUBE d={d:.2f}m pos=({self.px:.1f},{self.py:.1f})', throttle_duration_sec=1.0)
        if d < 0.4:
            self._stop(); self._go(DESCEND); return
        s = min(0.4, max(0.15, d*0.3))
        t = Twist()
        t.linear.x = (dx/d)*s
        t.linear.y = (dy/d)*s
        t.linear.z = ae * 0.3
        self.cmd.publish(t)

    # ── DESCEND until sonar <= 0.3m ───────────────────────────────────

    def _do_descend(self):
        self.get_logger().info(f'DESCEND sonar={self.sonar:.2f}m', throttle_duration_sec=0.5)
        if self.sonar <= 0.3:
            self._stop()
            self.attached = False
            self.attach_t = None
            self._go(WAIT)
            return
        dx = self.cube_x - self.px
        dy = self.cube_y - self.py
        t = Twist()
        t.linear.z = -0.1
        t.linear.x = dx * 0.3
        t.linear.y = dy * 0.3
        self.cmd.publish(t)

    # ── WAIT for attach ───────────────────────────────────────────────

    def _do_wait(self):
        self._stop()
        if not self.attached:
            self.get_logger().info(
                '⏳ ON THE CUBE. Run: ros2 topic pub /cube/attach std_msgs/msg/Empty {} --once',
                throttle_duration_sec=3.0)
            return
        e = time.time() - self.attach_t
        self.get_logger().info(f'Attached! Ascending in {2.5-e:.1f}s', throttle_duration_sec=0.5)
        if e >= 2.5:
            self._go(ASCEND)

    # ── ASCEND to search alt ──────────────────────────────────────────

    def _do_ascend(self):
        e = self.srch_alt - self.pz
        self.get_logger().info(f'ASCEND z={self.pz:.1f} → {self.srch_alt}m', throttle_duration_sec=1.0)
        if e < 0.3:
            self._stop()
            self.spin_accum = 0.0
            self.last_yaw = self.yaw
            self._go(SPIN)
            return
        t = Twist(); t.linear.z = min(0.3, e*0.5)
        self.cmd.publish(t)

    # ── SPIN 360 looking for person ───────────────────────────────────

    def _do_spin(self):
        # Track yaw change
        if self.last_yaw is not None:
            dy = self.yaw - self.last_yaw
            if dy > math.pi:  dy -= 2*math.pi
            if dy < -math.pi: dy += 2*math.pi
            self.spin_accum += abs(dy)
        self.last_yaw = self.yaw

        deg = math.degrees(self.spin_accum)
        self.get_logger().info(f'SPIN {deg:.0f}°/360°', throttle_duration_sec=1.0)

        # Person detected?
        if self.front:
            self.get_logger().info('Person spotted → GOTO_PERSON')
            self._stop(); self._go(GOTO_PERSON); return
        if self.bottom:
            self.get_logger().info('Person below → ALIGN')
            self._stop(); self._go(ALIGN); return

        # Keep spinning (restart after 360 if not found)
        if self.spin_accum >= 2*math.pi:
            self.spin_accum = 0.0

        t = Twist(); t.angular.z = 0.4
        self.cmd.publish(t)

    # ── APPROACH person using front cam ───────────────────────────────

    def _do_goto_person(self):
        if self.bottom:
            self._stop(); self._go(ALIGN); return
        if not self.front:
            self.get_logger().warn('Lost person → SPIN')
            self.spin_accum = 0.0; self.last_yaw = self.yaw
            self._go(SPIN); return

        d = self.front[0]
        ex = d.bbox.center.position.x - 320.0
        bw = d.bbox.size_x
        close = bw > 120
        self.get_logger().info(f'GOTO_PERSON ex={ex:+.0f} bw={bw:.0f}', throttle_duration_sec=1.0)

        t = Twist()
        t.linear.x  = 0.4 if not close else 0.0
        t.angular.z = -ex * 0.003
        self.cmd.publish(t)

    # ── ALIGN above person using bottom cam ───────────────────────────

    def _do_align(self):
        det = None
        if self.bottom:
            det = max(self.bottom, key=lambda d: d.bbox.size_x * d.bbox.size_y)
        elif self.front:
            det = max(self.front, key=lambda d: d.bbox.size_x * d.bbox.size_y)

        if not det:
            self.get_logger().warn('Lost person → SPIN')
            self.spin_accum = 0.0; self.last_yaw = self.yaw
            self._go(SPIN); return

        using_bot = self.bottom is not None
        ex = det.bbox.center.position.x - 320.0
        ey = det.bbox.center.position.y - 240.0

        cx = abs(ex) < 50
        cy = abs(ey) < 50 if using_bot else True

        self.get_logger().info(
            f'ALIGN {"bot" if using_bot else "frt"} ex={ex:+.0f} ey={ey:+.0f} cx={cx} cy={cy}',
            throttle_duration_sec=1.0)

        if cx and cy:
            self.get_logger().info('Centred → DROP_DESCEND')
            self._stop(); self._go(DROP_DESC); return

        t = Twist()
        if using_bot:
            t.linear.y = -ex * 0.0004
            t.linear.x =  ey * 0.0004
        else:
            t.linear.x  = 0.3
            t.angular.z = -ex * 0.003
        self.cmd.publish(t)

    # ── DESCEND to drop alt ───────────────────────────────────────────

    def _do_drop_desc(self):
        self.get_logger().info(f'DROP_DESC sonar={self.sonar:.2f}m → {self.drop_alt}m', throttle_duration_sec=0.5)
        if self.sonar <= self.drop_alt:
            self._stop()
            self.drop_t = None
            self._go(DROP)
            return
        det = max(self.bottom, key=lambda d: d.bbox.size_x*d.bbox.size_y) if self.bottom else None
        t = Twist()
        t.linear.z = -0.1
        if det:
            t.linear.y = -(det.bbox.center.position.x - 320) * 0.0003
            t.linear.x =  (det.bbox.center.position.y - 240) * 0.0003
        self.cmd.publish(t)

    # ── DROP ──────────────────────────────────────────────────────────

    def _do_drop(self):
        self._stop()
        if self.drop_t is None:
            self.get_logger().info('📦 DETACH — cube dropped!')
            self.detach.publish(Empty())
            self.drop_t = time.time()
            return
        if time.time() - self.drop_t >= 2.0:
            self._go(LAND)

    # ── LAND ──────────────────────────────────────────────────────────

    def _do_land(self):
        self._stop()
        if self.land_t is None:
            self.get_logger().info('🛬 Landing')
            self.lnd.publish(Empty())
            self.land_t = time.time()
            return
        if time.time() - self.land_t >= 5.0:
            self.get_logger().info('=== MISSION COMPLETE ===')
            self._go(IDLE)

    # ── helpers ───────────────────────────────────────────────────────

    def _stop(self):
        self.cmd.publish(Twist())

    def _go(self, s):
        self.state = s
        m = String(); m.data = s
        self.stpub.publish(m)
        self.get_logger().info(f'──── {s.upper()} ────')


def main(args=None):
    rclpy.init(args=args)
    n = Controller()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()