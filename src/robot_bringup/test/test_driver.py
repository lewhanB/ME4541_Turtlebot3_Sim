"""Exercise the Webots controller without a simulator or ROS graph transport."""

from copy import deepcopy
import math
from types import SimpleNamespace

from geometry_msgs.msg import Twist, TwistStamped
import pytest
from std_srvs.srv import SetBool, Trigger
from turtlebot3_msgs.srv import Sound

from robot_bringup import turtlebot3_driver as driver_module


class FakeSensor:
    def __init__(self):
        self.position = 0.0
        self.period = None

    def enable(self, period):
        self.period = period

    def getValue(self):
        return self.position


class FakeMotor:
    def __init__(self, maximum=6.67):
        self.sensor = FakeSensor()
        self.maximum = maximum
        self.velocity = 0.0
        self.position_target = None

    def getPositionSensor(self):
        return self.sensor

    def getMaxVelocity(self):
        return self.maximum

    def setPosition(self, value):
        self.position_target = value

    def setVelocity(self, value):
        self.velocity = value


class FakeRobot:
    def __init__(self):
        self.time = 0.0
        self.motors = [FakeMotor(), FakeMotor()]

    def getTime(self):
        return self.time

    def getBasicTimeStep(self):
        return 16

    def getDevice(self, name):
        return self.motors[0 if name == 'left wheel motor' else 1]

    def sample(self, time, left, right):
        self.time = time
        for motor, position in zip(self.motors, (left, right)):
            motor.sensor.position = position


class FakePublisher:
    def __init__(self, message_type):
        self.message_type = message_type
        self.messages = []

    def publish(self, message):
        self.messages.append(deepcopy(message))


class FakeNode:
    def __init__(self):
        self.publishers = {}
        self.subscriptions = {}
        self.services = {}
        self.warnings = []
        self.destroyed = False

    def destroy_node(self):
        self.destroyed = True

    def create_publisher(self, message_type, topic, qos):
        del qos
        publisher = FakePublisher(message_type)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, message_type, topic, callback, qos):
        del qos
        self.subscriptions[topic] = (message_type, callback)
        return self.subscriptions[topic]

    def create_service(self, service_type, topic, callback):
        self.services[topic] = (service_type, callback)
        return self.services[topic]

    def get_logger(self):
        return SimpleNamespace(warning=self.warnings.append)


class FakeBroadcaster:
    def __init__(self, node):
        del node
        self.transforms = []

    def sendTransform(self, transform):
        self.transforms.append(deepcopy(transform))


@pytest.fixture
def make_driver(monkeypatch):
    monkeypatch.setattr(driver_module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(driver_module.rclpy, 'spin_once', lambda *args, **kwargs: None)
    monkeypatch.setattr(driver_module, 'TransformBroadcaster', FakeBroadcaster)

    def make(properties=None):
        node, robot = FakeNode(), FakeRobot()
        monkeypatch.setattr(driver_module.rclpy, 'create_node', lambda *args, **kwargs: node)
        driver = driver_module.TurtleBot3Driver()
        driver.init(SimpleNamespace(robot=robot), properties or {})
        return driver, robot, node

    return make


def command(linear=0.1, angular=0.0, stamp=None, stamped=True):
    message = TwistStamped() if stamped else Twist()
    twist = message.twist if stamped else message
    twist.linear.x, twist.angular.z = linear, angular
    if stamp is not None:
        message.header.stamp = driver_module.TurtleBot3Driver._stamp(stamp)
    return message


def test_physical_interfaces_and_legacy_command_toggle(make_driver):
    driver, robot, node = make_driver()
    assert node.subscriptions['cmd_vel'][0] is TwistStamped
    assert set(node.publishers) == {
        'odom', 'joint_states', 'sensor_state', 'battery_state', 'magnetic_field'}
    assert set(node.services) == {'motor_power', 'reset', 'reset_odometry', 'sound'}
    assert all(math.isinf(motor.position_target) for motor in robot.motors)
    driver, robot, node = make_driver({'enable_stamped_cmd_vel': 'false'})
    assert node.subscriptions['cmd_vel'][0] is Twist
    node.subscriptions['cmd_vel'][1](command(stamped=False))
    driver.step()
    assert robot.motors[0].velocity == pytest.approx(0.1 / 0.033)


def test_plugin_disconnect_stops_motors_and_releases_only_owned_context(
        make_driver, monkeypatch):
    shutdowns = []
    monkeypatch.setattr(driver_module.rclpy, 'try_shutdown', lambda: shutdowns.append(True))
    driver, robot, node = make_driver()
    node.subscriptions['cmd_vel'][1](command())
    driver.step()
    assert robot.motors[0].velocity > 0.0
    driver.stop()
    assert node.destroyed
    assert all(motor.velocity == 0.0 for motor in robot.motors)
    assert not shutdowns
    driver.stop()  # Webots teardown may be repeated after a disconnect.
    assert not shutdowns

    initializations = []
    monkeypatch.setattr(driver_module.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(driver_module.rclpy, 'init', lambda **kwargs: initializations.append(kwargs))
    driver, robot, node = make_driver()
    assert initializations == [{
        'args': None,
        'signal_handler_options': driver_module.SignalHandlerOptions.NO,
    }]
    driver.stop()
    driver.stop()
    assert shutdowns == [True]


@pytest.mark.parametrize('stamp', [0.0, 4.0, 5.0, 6.0, 1791701251.618444])
def test_watchdog_uses_receipt_time_with_any_client_clock(make_driver, stamp):
    driver, robot, node = make_driver()
    robot.time = 5.0
    callback = node.subscriptions['cmd_vel'][1]
    callback(command(stamp=stamp))
    driver.step()
    assert robot.motors[0].velocity > 0.0
    assert not node.warnings
    robot.time = 5.49
    driver.step()
    assert robot.motors[0].velocity > 0.0
    robot.time = 5.501
    driver.step()
    assert [motor.velocity for motor in robot.motors] == [0.0, 0.0]


def test_new_receipt_refreshes_watchdog_and_nonfinite_command_stops(make_driver):
    driver, robot, node = make_driver()
    robot.time = 1.0
    callback = node.subscriptions['cmd_vel'][1]
    callback(command(stamp=1791701251.0))
    driver.step()
    assert robot.motors[0].velocity > 0.0
    robot.time = 1.4
    callback(command(stamp=1791701251.4))
    robot.time = 1.6
    driver.step()
    assert robot.motors[0].velocity > 0.0
    callback(command(linear=float('nan')))
    driver.step()
    assert [motor.velocity for motor in robot.motors] == [0.0, 0.0]


def test_speed_limits_preserve_requested_curvature(make_driver):
    driver, robot, node = make_driver()
    node.subscriptions['cmd_vel'][1](command(linear=0.44, angular=2.0))
    driver.step()
    left, right = [motor.velocity * driver.WHEEL_RADIUS for motor in robot.motors]
    linear = (left + right) * 0.5
    angular = (right - left) / driver.WHEEL_SEPARATION
    assert linear <= driver.MAX_LINEAR_SPEED
    assert abs(angular / linear) == pytest.approx(2.0 / 0.44)
    assert max(abs(motor.velocity) for motor in robot.motors) <= 6.67


def test_encoder_arc_odometry_and_hardware_joint_names(make_driver):
    driver, robot, node = make_driver()
    driver.step()
    robot.sample(0.1, 1.0, 2.0)
    driver.step()
    odom = node.publishers['odom'].messages[-1]
    rotation = 0.033 / 0.160
    radius = (1.5 * 0.033) / rotation
    assert odom.pose.pose.position.x == pytest.approx(radius * math.sin(rotation))
    assert odom.pose.pose.position.y == pytest.approx(radius * (1 - math.cos(rotation)))
    assert odom.twist.twist.linear.x == pytest.approx(1.5 * 0.033 / 0.1)
    assert odom.twist.twist.angular.z == pytest.approx(rotation / 0.1)
    assert odom.header.frame_id == 'odom'
    assert odom.child_frame_id == 'base_footprint'
    assert odom.header.stamp.sec == 0 and odom.header.stamp.nanosec == 100_000_000
    joints = node.publishers['joint_states'].messages[-1]
    assert joints.name == ['wheel_left_joint', 'wheel_right_joint']
    assert list(joints.position) == [1.0, 2.0]
    assert list(joints.velocity) == [10.0, 20.0]
    assert joints.header.frame_id == 'base_link'
    state = node.publishers['sensor_state'].messages[-1]
    assert state.left_encoder == round(4096 / math.tau)
    assert state.right_encoder == round(2 * 4096 / math.tau)
    assert not state.header.frame_id
    assert driver._tf_broadcaster.transforms[-1].child_frame_id == 'base_footprint'


def test_encoder_initialization_and_simulation_reset(make_driver):
    driver, robot, node = make_driver()
    robot.sample(0.0, float('nan'), float('nan'))
    driver.step()
    assert not node.publishers['odom'].messages
    robot.sample(0.1, 2.0, 2.0)
    driver.step()
    assert node.publishers['odom'].messages[-1].pose.pose.position.x == 0.0
    robot.sample(0.2, 3.0, 3.0)
    driver.step()
    assert node.publishers['odom'].messages[-1].pose.pose.position.x == pytest.approx(0.033)
    node.subscriptions['cmd_vel'][1](command())
    robot.sample(0.0, 0.0, 0.0)
    driver.step()
    assert node.publishers['odom'].messages[-1].pose.pose.position.x == 0.0
    assert [motor.velocity for motor in robot.motors] == [0.0, 0.0]
    robot.sample(0.1, 1.0, 1.0)
    driver.step()
    assert node.publishers['odom'].messages[-1].pose.pose.position.x == pytest.approx(0.033)


def test_motor_power_reset_and_unsupported_buzzer(make_driver):
    driver, robot, node = make_driver()
    driver.step()
    robot.sample(0.1, 1.0, 1.0)
    driver.step()
    response = node.services['motor_power'][1](SetBool.Request(data=False), SetBool.Response())
    assert response.success
    node.subscriptions['cmd_vel'][1](command())
    driver.step()
    assert all(motor.velocity == 0.0 for motor in robot.motors)
    node.services['motor_power'][1](SetBool.Request(data=True), SetBool.Response())
    driver.step()
    assert all(motor.velocity == 0.0 for motor in robot.motors)
    node.services['reset_odometry'][1](Trigger.Request(), Trigger.Response())
    driver.step()
    assert node.publishers['odom'].messages[-1].pose.pose.position.x == 0.0
    node.subscriptions['cmd_vel'][1](command())
    driver.step()
    assert robot.motors[0].velocity > 0.0
    node.services['reset'][1](Trigger.Request(), Trigger.Response())
    assert robot.motors[0].velocity == 0.0
    response = node.services['sound'][1](Sound.Request(value=1), Sound.Response())
    assert not response.success


def test_publish_rate_and_optional_transform(make_driver):
    driver, robot, node = make_driver({'enable_odom_tf': 'false'})
    for index in range(63):
        robot.time = index * 0.016
        driver.step()
    # Non-divisible simulator steps still produce the requested average 20 Hz.
    assert len(node.publishers['odom'].messages) == 20
    assert driver._tf_broadcaster is None
    battery = node.publishers['battery_state'].messages[-1]
    assert battery.voltage == 12.0 and battery.percentage == 1.0
    assert math.isnan(battery.current)
    assert not battery.header.frame_id
    assert node.publishers['magnetic_field'].messages[-1].header.frame_id == 'imu_link'


@pytest.mark.parametrize('properties', [
    {'command_timeout': '0'}, {'publish_rate': 'nan'},
    {'battery_percentage': '2'}, {'enable_stamped_cmd_vel': 'maybe'},
])
def test_invalid_properties_fail_at_initialization(make_driver, properties):
    with pytest.raises(ValueError):
        make_driver(properties)
