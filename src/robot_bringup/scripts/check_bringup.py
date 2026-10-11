#!/usr/bin/env python3
"""Check the running Burger ROS interface without moving it by default.

Run after sourcing install/setup.bash. Use --test-motion only in a simulation
ROS_DOMAIN_ID: it publishes a short, slow turn and then a stop on /cmd_vel.
"""

import argparse
from collections import deque
import math
import sys
import time
import xml.etree.ElementTree as ET

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import BatteryState, Imu, JointState, LaserScan, MagneticField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from turtlebot3_msgs.msg import SensorState


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def finite(*values):
    return all(math.isfinite(value) for value in values)


class BringupCheck(Node):
    """Collect data and validate only this interface, allowing unrelated topics."""

    TYPES = {
        '/scan': LaserScan, '/imu': Imu, '/magnetic_field': MagneticField,
        '/joint_states': JointState, '/sensor_state': SensorState,
        '/battery_state': BatteryState, '/odom': Odometry,
        '/tf': TFMessage, '/tf_static': TFMessage,
        '/robot_description': String, '/clock': Clock,
    }
    JOINTS = {'wheel_left_joint', 'wheel_right_joint'}

    def __init__(self, unstamped=False):
        super().__init__('check_turtlebot3_bringup')
        self.latest = {}
        self.counts = {topic: 0 for topic in self.TYPES}
        self.clocks = deque(maxlen=500)
        self.edges = {'/tf': set(), '/tf_static': set()}
        self.command_type = Twist if unstamped else TwistStamped
        self.command_publisher = self.create_publisher(
            self.command_type, '/cmd_vel', 10)
        latched = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions_ = [
            self.create_subscription(
                message_type, topic,
                lambda msg, name=topic: self.receive(name, msg),
                latched if topic in ('/tf_static', '/robot_description')
                else qos_profile_sensor_data)
            for topic, message_type in self.TYPES.items()
        ]

    def receive(self, topic, message):
        self.latest[topic] = message
        self.counts[topic] += 1
        if topic == '/clock':
            self.clocks.append(seconds(message.clock))
        elif topic in self.edges:
            self.edges[topic].update(
                (transform.header.frame_id, transform.child_frame_id)
                for transform in message.transforms)

    def errors(self):
        errors = []
        graph = dict(self.get_topic_names_and_types())
        for topic, message_type in self.TYPES.items():
            ros_type = message_type.__module__.split('.')[0] + '/msg/' + message_type.__name__
            if ros_type not in graph.get(topic, []):
                errors.append(f'{topic}: expected type {ros_type}; found {graph.get(topic, [])}')
            if topic not in self.latest:
                errors.append(f'{topic}: no message received')
        command_type = 'geometry_msgs/msg/' + self.command_type.__name__
        subscribers = self.get_subscriptions_info_by_topic('/cmd_vel')
        if not any(endpoint.topic_type == command_type for endpoint in subscribers):
            errors.append(f'/cmd_vel: no {command_type} subscriber')
        if len(self.clocks) < 2 or max(self.clocks, default=0) <= min(self.clocks, default=0):
            errors.append('/clock: simulation time is not advancing (is Webots paused?)')
        if not self.TYPES.keys() <= self.latest.keys():
            return errors

        clock = self.clocks[-1]
        for topic in ('/scan', '/imu', '/magnetic_field', '/joint_states',
                      '/sensor_state', '/battery_state', '/odom'):
            stamp = seconds(self.latest[topic].header.stamp)
            if not -1.0 <= clock - stamp <= 2.0:
                errors.append(f'{topic}: stamp {stamp:.3f} differs from simulation clock {clock:.3f}')

        scan = self.latest['/scan']
        if (scan.header.frame_id != 'base_scan' or not scan.ranges
                or scan.range_min < 0 or scan.range_max <= scan.range_min
                or not finite(scan.angle_min, scan.angle_max, scan.angle_increment)
                or scan.angle_increment == 0
                or not math.isclose(
                    scan.angle_max - scan.angle_min,
                    scan.angle_increment * (len(scan.ranges) - 1),
                    rel_tol=0.01, abs_tol=1e-4)
                or any(math.isnan(v) for v in scan.ranges)):
            errors.append('/scan: invalid laser geometry, ranges, or base_scan frame')
        imu = self.latest['/imu']
        q = imu.orientation
        if (imu.header.frame_id != 'imu_link'
                or not finite(q.x, q.y, q.z, q.w,
                              imu.angular_velocity.x, imu.angular_velocity.y,
                              imu.angular_velocity.z, imu.linear_acceleration.x,
                              imu.linear_acceleration.y, imu.linear_acceleration.z)
                or abs(sum(v * v for v in (q.x, q.y, q.z, q.w)) - 1.0) > 0.01):
            errors.append('/imu: nonfinite data, invalid quaternion, or imu_link frame')
        magnetic = self.latest['/magnetic_field']
        if (magnetic.header.frame_id != 'imu_link'
                or not finite(magnetic.magnetic_field.x, magnetic.magnetic_field.y,
                              magnetic.magnetic_field.z)):
            errors.append('/magnetic_field: invalid data or imu_link frame')
        joints = self.latest['/joint_states']
        if (not self.JOINTS <= set(joints.name)
                or len(joints.position) != len(joints.name)
                or not finite(*joints.position, *joints.velocity)):
            errors.append('/joint_states: missing hardware wheel joint names or invalid data')
        battery = self.latest['/battery_state']
        if (not finite(battery.voltage, battery.percentage)
                or battery.voltage <= 0 or not 0 <= battery.percentage <= 1
                or not battery.present):
            errors.append('/battery_state: invalid battery voltage, percentage, or presence')
        sensors = self.latest['/sensor_state']
        if not math.isfinite(sensors.battery) or sensors.battery <= 0:
            errors.append('/sensor_state: invalid battery voltage')
        odom = self.latest['/odom']
        if (odom.header.frame_id != 'odom' or odom.child_frame_id != 'base_footprint'
                or not finite(odom.pose.pose.position.x, odom.pose.pose.position.y,
                              odom.twist.twist.linear.x, odom.twist.twist.angular.z)):
            errors.append('/odom: invalid frames or nonfinite pose/twist')
        if ('odom', 'base_footprint') not in self.edges['/tf']:
            errors.append('/tf: missing odom -> base_footprint transform')
        for edge in (('base_footprint', 'base_link'), ('base_link', 'imu_link'),
                     ('base_link', 'base_scan')):
            if edge not in self.edges['/tf_static']:
                errors.append(f'/tf_static: missing {edge[0]} -> {edge[1]} transform')
        try:
            robot = ET.fromstring(self.latest['/robot_description'].data)
            links = {link.get('name') for link in robot.findall('link')}
            joints = {joint.get('name') for joint in robot.findall('joint')}
            if not {'base_footprint', 'base_link', 'base_scan', 'imu_link'} <= links:
                errors.append('/robot_description: missing hardware frame links')
            if not self.JOINTS <= joints:
                errors.append('/robot_description: missing hardware wheel joints')
        except ET.ParseError as error:
            errors.append(f'/robot_description: invalid XML: {error}')
        return errors

    def send_turn(self, speed):
        message = self.command_type()
        twist = message.twist if isinstance(message, TwistStamped) else message
        if isinstance(message, TwistStamped) and self.clocks:
            message.header.stamp = self.latest['/clock'].clock
        twist.angular.z = speed
        self.command_publisher.publish(message)

    def motion_check(self):
        """Turn slowly for one simulation second; verify motion and watchdog stop."""
        before = self.latest['/joint_states']
        starting_positions = dict(zip(before.name, before.position))
        start = self.clocks[-1]
        deadline = time.monotonic() + 6.0
        last_publish = 0.0
        commanded = False
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.02)
                elapsed = self.clocks[-1] - start
                wall_now = time.monotonic()
                if elapsed < 1.0 and wall_now - last_publish >= 0.05:
                    self.send_turn(0.25)
                    last_publish = wall_now
                    commanded = True
                if elapsed >= 2.0:
                    joints = self.latest['/joint_states']
                    positions = dict(zip(joints.name, joints.position))
                    moved = any(abs(positions[name] - starting_positions[name]) > 0.05
                                for name in self.JOINTS)
                    odom = self.latest['/odom']
                    stopped = (abs(odom.twist.twist.linear.x) < 0.01
                               and abs(odom.twist.twist.angular.z) < 0.02)
                    errors = []
                    if not moved:
                        errors.append('motion: wheel encoders did not respond to /cmd_vel')
                    if not stopped:
                        errors.append('motion: command watchdog did not stop the wheels')
                    return errors
            return ['motion: simulation did not advance two seconds within six wall seconds']
        finally:
            if commanded:
                # Also stop explicitly when the checker is interrupted or times out.
                for _ in range(3):
                    self.send_turn(0.0)
                    rclpy.spin_once(self, timeout_sec=0.03)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=20.0,
                        help='maximum observation time in wall seconds (default: 20)')
    parser.add_argument('--unstamped', action='store_true',
                        help='expect geometry_msgs/Twist instead of TwistStamped on /cmd_vel')
    parser.add_argument('--test-motion', action='store_true',
                        help='publish a slow turn; use only in an isolated simulation ROS domain')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    rclpy.init(args=[])
    node = BringupCheck(args.unstamped)
    errors = []
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(node.counts.values()) and len(node.clocks) >= 2:
                errors = node.errors()
                if not errors:
                    break
        else:
            errors = node.errors()
        for topic in node.TYPES:
            print(f'{topic}: {node.counts[topic]} message(s)')
        if args.test_motion and not errors:
            print('Testing /cmd_vel response and the simulation-time command watchdog...')
            errors.extend(node.motion_check())
        if errors:
            for error in errors:
                print(f'FAIL: {error}', file=sys.stderr)
            return 1
        print('PASS: Burger bringup topics, frames, sensor data, and simulation stamps')
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
