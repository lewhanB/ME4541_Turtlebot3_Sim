"""Webots wheel controller exposing the TurtleBot3 Burger bringup interfaces.

Wheel encoders drive odometry; the native Webots plugins supply IMU and lidar.
Battery voltage/percentage are configurable ideal values.
Magnetic field is an ideal world field rotated by encoder heading.
Unmodelled bumper/cliff/sonar/button values in SensorState stay zero.
"""

import math

from builtin_interfaces.msg import Time

from geometry_msgs.msg import TransformStamped, Twist, TwistStamped

from nav_msgs.msg import Odometry

import rclpy
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from sensor_msgs.msg import BatteryState, JointState, MagneticField

from std_srvs.srv import SetBool, Trigger

from tf2_ros import TransformBroadcaster

from turtlebot3_msgs.msg import SensorState
from turtlebot3_msgs.srv import Sound


class TurtleBot3Driver:
    """Control the robot using Webots simulation time and wheel encoders."""

    WHEEL_RADIUS = 0.033
    WHEEL_SEPARATION = 0.160
    MAX_LINEAR_SPEED = 0.22
    MAX_ANGULAR_SPEED = 2.84
    ENCODER_TICKS_PER_REVOLUTION = 4096
    JOINT_NAMES = ['wheel_left_joint', 'wheel_right_joint']

    def init(self, webots_node, properties):
        """Configure devices and ROS interfaces once the driver connects."""
        self._robot = webots_node.robot
        self._command_timeout = self._positive_property(
            properties, 'command_timeout', 0.5)
        self._publish_period = 1.0 / self._positive_property(
            properties, 'publish_rate', 20.0)
        self._enable_stamped_cmd_vel = self._bool_property(
            properties, 'enable_stamped_cmd_vel', True)
        self._enable_odom_tf = self._bool_property(
            properties, 'enable_odom_tf', True)
        self._battery_voltage = self._positive_property(
            properties, 'battery_voltage', 12.0)
        self._battery_percentage = float(
            properties.get('battery_percentage', 1.0))
        if not 0.0 <= self._battery_percentage <= 1.0:
            raise ValueError('battery_percentage must be between 0 and 1')

        self._owns_ros_context = not rclpy.ok()
        if self._owns_ros_context:
            # Webots' C++ host owns process signals and calls our stop hook.
            rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
        self._node = rclpy.create_node(
            'turtlebot3_node',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
            use_global_arguments=False,
        )
        self._motors = [
            self._robot.getDevice('left wheel motor'),
            self._robot.getDevice('right wheel motor'),
        ]
        self._sensors = []
        self._max_wheel_speeds = []
        timestep = max(1, round(self._robot.getBasicTimeStep()))
        for motor in self._motors:
            if motor is None:
                raise RuntimeError('TurtleBot3 world is missing a wheel motor')
            sensor = motor.getPositionSensor()
            if sensor is None:
                raise RuntimeError(
                    'TurtleBot3 wheel motor has no position sensor')
            maximum = float(motor.getMaxVelocity())
            if not math.isfinite(maximum) or maximum <= 0.0:
                raise RuntimeError(
                    'TurtleBot3 wheel motor has invalid maxVelocity')
            self._sensors.append(sensor)
            self._max_wheel_speeds.append(maximum)
            sensor.enable(timestep)
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

        self._odom_publisher = self._node.create_publisher(
            Odometry, 'odom', 10)
        self._joint_publisher = self._node.create_publisher(
            JointState, 'joint_states', 10)
        self._sensor_publisher = self._node.create_publisher(
            SensorState, 'sensor_state', 10)
        self._battery_publisher = self._node.create_publisher(
            BatteryState, 'battery_state', 10)
        self._magnetic_publisher = self._node.create_publisher(
            MagneticField, 'magnetic_field', 10)
        self._tf_broadcaster = (
            TransformBroadcaster(self._node) if self._enable_odom_tf else None)
        command_type = TwistStamped if self._enable_stamped_cmd_vel else Twist
        self._command_subscription = self._node.create_subscription(
            command_type, 'cmd_vel', self._on_command, 10)
        self._services = [
            self._node.create_service(
                SetBool, 'motor_power', self._on_motor_power),
            self._node.create_service(Trigger, 'reset', self._on_reset),
            self._node.create_service(
                Trigger, 'reset_odometry', self._on_reset_odometry),
            self._node.create_service(Sound, 'sound', self._on_sound),
        ]

        self._drive_enabled = True
        self._wheel_command = [0.0, 0.0]
        self._last_command_time = None
        self._last_step_time = None
        self._last_positions = None
        self._last_sample_time = None
        self._next_publish_time = None
        self._x = self._y = self._heading = 0.0
        self._linear_velocity = self._angular_velocity = 0.0
        self._joint_velocities = [0.0, 0.0]

    def stop(self):
        """Release the Python plugin when Webots disconnects the controller."""
        for motor in getattr(self, '_motors', []):
            if motor is not None:
                motor.setVelocity(0.0)
        node = getattr(self, '_node', None)
        if node is not None:
            node.destroy_node()
            self._node = None
        if getattr(self, '_owns_ros_context', False):
            rclpy.try_shutdown()
            self._owns_ros_context = False

    @staticmethod
    def _bool_property(properties, name, default):
        value = str(properties.get(name, default)).strip().lower()
        if value not in ('true', 'false', '1', '0'):
            raise ValueError(f'{name} must be true or false')
        return value in ('true', '1')

    @staticmethod
    def _positive_property(properties, name, default):
        value = float(properties.get(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{name} must be a positive finite number')
        return value

    @staticmethod
    def _stamp(seconds):
        nanoseconds = round(seconds * 1_000_000_000)
        return Time(sec=nanoseconds // 1_000_000_000,
                    nanosec=nanoseconds % 1_000_000_000)

    def _clear_command(self):
        self._wheel_command = [0.0, 0.0]
        self._last_command_time = None
        for motor in self._motors:
            motor.setVelocity(0.0)

    def _on_command(self, message):
        now = float(self._robot.getTime())
        twist = message.twist if self._enable_stamped_cmd_vel else message
        linear, angular = float(twist.linear.x), float(twist.angular.z)
        if not all(math.isfinite(value) for value in (now, linear, angular)):
            self._clear_command()
            self._node.get_logger().warning(
                'Rejected a nonfinite velocity command')
            return
        if not self._drive_enabled:
            self._clear_command()
            return

        scale = max(1.0, abs(linear) / self.MAX_LINEAR_SPEED,
                    abs(angular) / self.MAX_ANGULAR_SPEED)
        linear, angular = linear / scale, angular / scale
        half_track_velocity = angular * self.WHEEL_SEPARATION * 0.5
        left = (linear - half_track_velocity) / self.WHEEL_RADIUS
        right = (linear + half_track_velocity) / self.WHEEL_RADIUS
        wheel_scale = max(1.0, abs(left) / self._max_wheel_speeds[0],
                          abs(right) / self._max_wheel_speeds[1])
        self._wheel_command = [
            max(-maximum, min(maximum, speed / wheel_scale))
            for speed, maximum in zip((left, right), self._max_wheel_speeds)]
        # Match the TurtleBot3 hardware receiver: consume the twist on receipt.
        # Stock turtlebot3_teleop stamps with a standalone system Clock(), even
        # when use_sim_time is set. Use Webots time for the receipt watchdog so
        # both that teleop and simulation-clock clients work on /cmd_vel.
        self._last_command_time = now

    def _reset_odometry(self):
        self._x = self._y = self._heading = 0.0
        self._linear_velocity = self._angular_velocity = 0.0
        self._joint_velocities = [0.0, 0.0]
        readings = [float(sensor.getValue()) for sensor in self._sensors]
        self._last_positions = (
            readings if all(map(math.isfinite, readings)) else None)
        self._last_sample_time = float(self._robot.getTime())
        self._next_publish_time = self._last_sample_time

    def _on_motor_power(self, request, response):
        self._drive_enabled = bool(request.data)
        self._clear_command()
        response.success = True
        response.message = (
            'Wheel motors enabled' if self._drive_enabled
            else 'Wheel motors disabled')
        return response

    def _on_reset(self, request, response):
        self._clear_command()
        return self._on_reset_odometry(request, response)

    def _on_reset_odometry(self, request, response):
        del request
        self._reset_odometry()
        response.success = True
        response.message = 'Encoder odometry reset'
        return response

    @staticmethod
    def _on_sound(request, response):
        del request
        response.success = False
        response.message = 'The Webots Burger model has no buzzer'
        return response

    def step(self):
        """Process callbacks, drive the wheels, and publish encoder state."""
        now = float(self._robot.getTime())
        if not math.isfinite(now) or now < 0.0:
            self._clear_command()
            return
        if self._last_step_time is not None and now < self._last_step_time:
            self._clear_command()
            self._reset_odometry()
        self._last_step_time = now
        rclpy.spin_once(self._node, timeout_sec=0)
        active = (
            self._drive_enabled and self._last_command_time is not None
            and 0.0 <= now - self._last_command_time <= self._command_timeout)
        for motor, speed in zip(self._motors, self._wheel_command):
            motor.setVelocity(speed if active else 0.0)

        positions = [float(sensor.getValue()) for sensor in self._sensors]
        if not all(map(math.isfinite, positions)):
            # Encoder continuity across missing readings is unknown.
            self._last_positions = None
            self._last_sample_time = None
            return
        if self._last_positions is not None and now > self._last_sample_time:
            dt = now - self._last_sample_time
            delta = [
                position - previous
                for position, previous in zip(positions, self._last_positions)]
            self._joint_velocities = [value / dt for value in delta]
            left, right = [value * self.WHEEL_RADIUS for value in delta]
            distance = 0.5 * (left + right)
            rotation = (right - left) / self.WHEEL_SEPARATION
            # Midpoint/sinc integration is stable for arcs and tiny turns.
            half_rotation = 0.5 * rotation
            sinc = (math.sin(half_rotation) / half_rotation
                    if abs(half_rotation) > 1e-9 else 1.0)
            middle_heading = self._heading + half_rotation
            self._x += distance * sinc * math.cos(middle_heading)
            self._y += distance * sinc * math.sin(middle_heading)
            self._heading = math.atan2(math.sin(self._heading + rotation),
                                       math.cos(self._heading + rotation))
            self._linear_velocity = distance / dt
            self._angular_velocity = rotation / dt
        elif self._last_positions is None:
            self._linear_velocity = self._angular_velocity = 0.0
            self._joint_velocities = [0.0, 0.0]
        self._last_positions = positions
        self._last_sample_time = now
        if self._next_publish_time is None:
            self._next_publish_time = now
        if now + 1e-9 >= self._next_publish_time:
            self._publish(now, positions)
            elapsed_periods = (
                (now - self._next_publish_time) / self._publish_period)
            periods = max(1, math.floor(elapsed_periods) + 1)
            self._next_publish_time += periods * self._publish_period

    def _publish(self, now, positions):
        stamp = self._stamp(now)
        sin_heading = math.sin(self._heading)
        cos_heading = math.cos(self._heading)
        qz, qw = math.sin(self._heading * 0.5), math.cos(self._heading * 0.5)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self._x, self._y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x = self._linear_velocity
        odom.twist.twist.angular.z = self._angular_velocity
        for index, value in ((0, 1e-3), (7, 1e-3), (14, 1e6),
                             (21, 1e6), (28, 1e6), (35, 1e-3)):
            odom.pose.covariance[index] = value
            odom.twist.covariance[index] = value
        self._odom_publisher.publish(odom)
        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            transform.transform.translation.x = self._x
            transform.transform.translation.y = self._y
            transform.transform.rotation = odom.pose.pose.orientation
            self._tf_broadcaster.sendTransform(transform)

        joints = JointState()
        joints.header.stamp = stamp
        joints.header.frame_id = 'base_link'
        joints.name = self.JOINT_NAMES.copy()
        joints.position = positions
        joints.velocity = self._joint_velocities
        joints.effort = [0.0, 0.0]
        self._joint_publisher.publish(joints)

        sensors = SensorState()
        sensors.header.stamp = stamp
        sensors.torque = self._drive_enabled
        # OpenCR publishes signed int32 encoder counters, wrapping on overflow.
        counts = [round(value / math.tau * self.ENCODER_TICKS_PER_REVOLUTION)
                  for value in positions]
        sensors.left_encoder, sensors.right_encoder = [
            (count + (1 << 31)) % (1 << 32) - (1 << 31) for count in counts]
        sensors.battery = self._battery_voltage
        self._sensor_publisher.publish(sensors)

        battery = BatteryState()
        battery.header.stamp = stamp
        battery.voltage = self._battery_voltage
        battery.percentage = self._battery_percentage
        battery.present = True
        battery.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        battery.power_supply_technology = (
            BatteryState.POWER_SUPPLY_TECHNOLOGY_LION)
        for field in (
                'temperature', 'current', 'charge', 'capacity', 'design_capacity'):
            setattr(battery, field, float('nan'))
        self._battery_publisher.publish(battery)

        magnetic = MagneticField()
        magnetic.header.stamp = stamp
        magnetic.header.frame_id = 'imu_link'
        magnetic.magnetic_field.x = 20e-6 * cos_heading
        magnetic.magnetic_field.y = -20e-6 * sin_heading
        magnetic.magnetic_field.z = 40e-6
        self._magnetic_publisher.publish(magnetic)
