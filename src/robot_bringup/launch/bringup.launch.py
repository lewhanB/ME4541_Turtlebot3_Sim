"""Run the TurtleBot3 Burger simulation, publishing the hardware interfaces."""

import os
from pathlib import Path
import shlex
import sys
import tempfile
from unittest.mock import patch
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher

from robot_bringup.wsl_network import windows_host


def _launch(context):
    share = Path(get_package_share_directory('robot_bringup'))
    description_share = Path(get_package_share_directory('robot_description'))
    world = Path(LaunchConfiguration('world').perform(context)).expanduser()
    if not world.is_absolute():
        world = share / 'worlds' / world
    if not world.is_file():
        raise ValueError(f'Webots world does not exist: {world}')
    port = LaunchConfiguration('port').perform(context)
    if not port.isdigit() or not 1 <= int(port) <= 65535:
        raise ValueError('port must be between 1 and 65535')
    host = windows_host(LaunchConfiguration('webots_host').perform(context))

    # Keep runtime overrides out of the installed source files. The driver API
    # accepts a URDF path, and its plugin properties come from this document.
    runtime = tempfile.TemporaryDirectory(prefix='turtlebot3_webots_')
    config = ET.parse(share / 'config' / 'webots.urdf')
    plugin = config.find('./webots/plugin[@type="robot_bringup.turtlebot3_driver.TurtleBot3Driver"]')
    for name in ('enable_stamped_cmd_vel', 'enable_odom_tf', 'command_timeout'):
        plugin.find(name).text = LaunchConfiguration(name).perform(context)
    config_path = Path(runtime.name) / 'webots.urdf'
    config.write(config_path, encoding='unicode', xml_declaration=True)

    # webots_ros2 2025.0.0 has no host argument and derives the host from DNS.
    # Override only while constructing the two controller commands; restore the
    # installed helper immediately afterwards. No system files are changed.
    with patch('webots_ros2_driver.utils.get_wsl_ip_address', return_value=host):
        webots = WebotsLauncher(
            world=str(world), mode=LaunchConfiguration('mode').perform(context),
            gui=LaunchConfiguration('gui'), port=port, ros2_supervisor=True,
        )
        driver = WebotsController(
            robot_name='TurtleBot3Burger', port=port,
            parameters=[{
                'robot_description': str(config_path),
                'use_sim_time': True,
                'set_robot_state_publisher': False,
            }],
            prefix=shlex.join([
                sys.executable, str(share / 'scripts' / 'controller_process.py')]),
            respawn=False, sigterm_timeout='4', sigkill_timeout='2',
        )

    def cleanup(_context):
        runtime.cleanup()
        return []

    return [
        LogInfo(msg=f'Windows Webots controller endpoint: {host}:{port}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(description_share / 'launch' / 'display.launch.py')),
            launch_arguments={
                'use_sim_time': 'true', 'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        RegisterEventHandler(OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=Shutdown(reason='Webots exited'))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=driver,
            on_exit=[EmitEvent(event=Shutdown(reason='TurtleBot3 controller exited'))],
        )),
        webots,
        webots._supervisor,
        driver,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='robot_world.wbt',
                             description='Bundled world filename or absolute .wbt path.'),
        DeclareLaunchArgument('webots_host', default_value=os.environ.get('WEBOTS_HOST', 'auto'),
                             description='auto detects NAT gateway or mirrored localhost.'),
        DeclareLaunchArgument('port', default_value='1234', description='Webots controller TCP port.'),
        DeclareLaunchArgument('mode', default_value='realtime', choices=['realtime', 'fast', 'pause']),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('enable_stamped_cmd_vel', default_value='true',
                             choices=['true', 'false'], description='Jazzy hardware default is TwistStamped.'),
        DeclareLaunchArgument('enable_odom_tf', default_value='true', choices=['true', 'false'],
                             description='Set false when an EKF publishes odom -> base_footprint.'),
        DeclareLaunchArgument('command_timeout', default_value='0.5',
                             description='Stop the wheels after this many simulation seconds without a command.'),
        OpaqueFunction(function=_launch),
    ])
