#!/usr/bin/env python3
"""
Payload Manager - Gazebo Harmonic
Uses /world/<world_name>/set_pose service to teleport cube.
This is the correct Gazebo Harmonic API.

When attached: cube teleports to drone position every 0.1s
When detached: cube stays at last position, falls with gravity
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose
import math


class PayloadManager(Node):
    def __init__(self):
        super().__init__('payload_manager')

        self.declare_parameter('payload_name', 'red_delivery_cube')
        self.declare_parameter('odom_topic',   '/simple_drone/odom')
        self.declare_parameter('world_name',   'empty')
        self.declare_parameter('offset_z',     -0.25)

        self.payload_name = self.get_parameter('payload_name').value
        odom_topic        = self.get_parameter('odom_topic').value
        world_name        = self.get_parameter('world_name').value
        self.offset_z     = self.get_parameter('offset_z').value

        self.attached = False
        self.drone_x  = 0.0
        self.drone_y  = 0.0
        self.drone_z  = 0.0

        # Publishers
        self.status_pub   = self.create_publisher(String, '/payload/status',   10)
        self.attached_pub = self.create_publisher(Bool,   '/payload/attached', 10)

        # Services exposed
        self.create_service(Empty, '/payload/attach', self._attach_cb)
        self.create_service(Empty, '/payload/detach', self._detach_cb)

        # Gazebo Harmonic set pose service
        self.set_pose_cli = self.create_client(
            SetEntityPose,
            f'/world/{world_name}/set_pose'
        )

        # Odom subscriber
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        # Follow timer 10 Hz
        self.create_timer(0.1, self._follow)

        self.get_logger().info('Payload Manager ready')
        self.get_logger().info(f'  payload : {self.payload_name}')
        self.get_logger().info(f'  service : /world/{world_name}/set_pose')

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.drone_x = p.x
        self.drone_y = p.y
        self.drone_z = p.z

    def _attach_cb(self, req, res):
        self.attached = True
        self.get_logger().info(f'Attached {self.payload_name}')
        self._pub(True)
        return res

    def _detach_cb(self, req, res):
        self.attached = False
        self.get_logger().info(f'Detached — {self.payload_name} dropped!')
        self._pub(False)
        return res

    def _follow(self):
        if not self.attached:
            return
        if not self.set_pose_cli.service_is_ready():
            return

        pose = Pose()
        pose.position.x = self.drone_x
        pose.position.y = self.drone_y
        pose.position.z = self.drone_z + self.offset_z
        pose.orientation.w = 1.0

        req = SetEntityPose.Request()
        req.entity.name = self.payload_name
        req.entity.type = 2   # 2 = MODEL
        req.pose = pose
        self.set_pose_cli.call_async(req)

    def _pub(self, attached):
        s = String(); s.data = 'attached' if attached else 'detached'
        self.status_pub.publish(s)
        b = Bool(); b.data = attached
        self.attached_pub.publish(b)


def main(args=None):
    rclpy.init(args=args)
    node = PayloadManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()