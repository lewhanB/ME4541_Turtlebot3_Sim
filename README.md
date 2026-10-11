# SLIIT - ME4541 TurtleBot3 Simulation

A ROS 2 Jazzy workspace for simulating a **TurtleBot3 Burger** in Webots with
the topic names, message types, and robot frames used by the hardware bringup.
The default launch opens the furnished `robot_world.wbt` world, the robot
controller, simulation clock, and robot state publisher. RViz is optional.

Use `~/turtlebot3_ws` as the colcon workspace. Its source layout is:

```text
turtlebot3_ws/
  src/
    robot_description/  # Burger URDF, meshes, robot state publisher, RViz config
    robot_bringup/      # Webots worlds, controller, launch files, interface checks
```

## Requirements

The tested setup is **Ubuntu 24.04 in WSL2**, **ROS 2 Jazzy**, and
**Webots R2025a installed on Windows**, using WSL2 NAT networking. ROS nodes run
inside WSL; Windows runs Webots. Mirrored networking is also handled by the
launcher's host detection. Native Linux uses the standard Webots launcher
backend but has not been validated for this workspace.

1. Install [WSL2 with Ubuntu 24.04](https://learn.microsoft.com/en-us/windows/wsl/install).
2. Follow the [ROS 2 Jazzy Ubuntu installation instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
   inside WSL, including the ROS apt repository setup. Install `ros-jazzy-desktop`
   for ROS and RViz.
3. Install [Webots R2025a for Windows](https://github.com/cyberbotics/webots/releases/tag/R2025a).
   The [official Webots ROS 2 Windows guide](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Installation-Windows.html)
   describes the WSL/Windows arrangement.

After ROS is installed, run these commands inside WSL:

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-rosdep
source /opt/ros/jazzy/setup.bash
```

Initialize rosdep **once per machine**. Skip `sudo rosdep init` if it has already
been initialized; run `rosdep update` as your regular user:

```bash
sudo rosdep init
rosdep update
```

## Clone and build

Create a new workspace at `~/turtlebot3_ws` using Git sparse checkout to
retrieve only the `src/` directory. The repository's top-level README, LICENSE,
and other root files are not checked out. Package licenses and notices inside
`src/` remain with the packages. Git retains its `.git/` metadata for updates.

Run these commands in WSL; the destination should not already contain a clone:

```bash
git clone --filter=blob:none --no-checkout --depth 1 \
  https://github.com/lewhanB/ME4541_Turtlebot3_Sim.git ~/turtlebot3_ws
cd ~/turtlebot3_ws
git sparse-checkout set --no-cone '/src/'
git checkout main
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
```

Rosdep installs the declared ROS dependencies, including `webots_ros2_driver`,
`turtlebot3_msgs`, and `turtlebot3_teleop`. The Windows Webots application is
installed separately as described above; robot hardware packages and devices
are not needed to run this simulation.

## Launch

Run in a WSL terminal, adjusting `WEBOTS_HOME` if Webots is installed elsewhere:

```bash
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export WEBOTS_HOME='/mnt/c/Program Files/Webots'
ros2 launch robot_bringup bringup.launch.py
```

Keep the export in each new bringup terminal or add it to your own shell startup
configuration. If you already have a working `WEBOTS_HOME` export, keep it.
`ROS2_WEBOTS_HOME`, when set, takes precedence over `WEBOTS_HOME` in the upstream
Webots driver.

`ros2 launch robot_bringup robot.launch.py` is an equivalent entry point.
To open RViz too:

```bash
ros2 launch robot_bringup bringup.launch.py rviz:=true
```

The default world is `src/robot_bringup/worlds/robot_world.wbt`, including its
furnished environment and TurtleBot3 Burger. Webots may download missing remote
PROTOs and their assets on first launch. For an arena with bundled robot assets
and no world asset downloads:

```bash
ros2 launch robot_bringup bringup.launch.py world:=burger_arena.wbt
```

Use `ros2 launch robot_bringup bringup.launch.py --show-args` for all options.
Only the Burger model is implemented; no `TURTLEBOT3_MODEL` or `LDS_MODEL` export
is required for this simulation.

## WSL2 networking

The default `webots_host:=auto` checks the active WSL networking mode at startup.
For **NAT** it uses the default gateway from the routing table. For **mirrored**
networking it uses `127.0.0.1`. It does not mistake a DNS server for the Windows host.
After changing `.wslconfig`, run `wsl --shutdown` from Windows PowerShell and
reopen your WSL terminal for the change to take effect. See
[Microsoft's WSL networking documentation](https://learn.microsoft.com/en-us/windows/wsl/networking).

If detection fails, read the gateway using `ip route show default` and pass it:

```bash
ros2 launch robot_bringup bringup.launch.py webots_host:=172.28.208.1
```

Replace the example address with your current gateway. `WEBOTS_HOST` can also
set the default. The TCP port defaults to `1234`; change it with `port:=1235`.
If the controller waits for Webots, check the logged endpoint and allow the
Windows Webots application through Windows Firewall on the WSL network.
All ROS nodes run inside WSL; Windows runs the simulator.

## Hardware-compatible ROS interface

Topic names, message types, frame IDs and wheel joint names follow the
ROBOTIS TurtleBot3 Jazzy bringup. Published timestamps and the command watchdog use
simulation time. The default subscription matches Jazzy's
`enable_stamped_cmd_vel: true`.

| Topic | Message type | Frame / purpose |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Velocity input |
| `/scan` | `sensor_msgs/msg/LaserScan` | `base_scan`, 5 Hz |
| `/imu` | `sensor_msgs/msg/Imu` | `imu_link`, 20 Hz |
| `/magnetic_field` | `sensor_msgs/msg/MagneticField` | `imu_link`, 20 Hz |
| `/joint_states` | `sensor_msgs/msg/JointState` | `wheel_left_joint`, `wheel_right_joint`, 20 Hz |
| `/sensor_state` | `turtlebot3_msgs/msg/SensorState` | Encoders, motor enable state, battery, 20 Hz |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | Ideal battery, 20 Hz |
| `/odom` | `nav_msgs/msg/Odometry` | `odom` → `base_footprint`, encoder odometry, 20 Hz |
| `/tf` | `tf2_msgs/msg/TFMessage` | Odometry and moving wheel transforms |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Fixed robot transforms |
| `/robot_description` | `std_msgs/msg/String` | Local Burger URDF, transient local |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation clock |

Webots also provides infrastructure topics, including `/scan/point_cloud` and
supervisor topics; ROS provides `/rosout` and `/parameter_events`.
Services: `/motor_power` (`std_srvs/srv/SetBool`), `/reset` and `/reset_odometry`
(`std_srvs/srv/Trigger`), `/sound` (`turtlebot3_msgs/srv/Sound`). Reset services
reset the odometry origin, not the robot's world position. Sound reports that
the model has no buzzer.

For a client that publishes `Twist`, use
`enable_stamped_cmd_vel:=false`; this changes the type of `/cmd_vel`.
When an EKF owns the `odom` → `base_footprint` transform, use
`enable_odom_tf:=false`.

Example stamped command, in a second sourced terminal:

```bash
ros2 topic pub --use-sim-time --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: auto, twist: {linear: {x: 0.1}, angular: {z: 0.2}}}'
```

Commands time out `0.5` simulation seconds after the last command is received.
The driver consumes the twist on receipt, like the hardware TurtleBot3 receiver;
it does not compare the command header against the simulator clock. This accepts
system-clock, simulation-clock, and zero-stamped commands. Stop the publisher
to stop the robot after the timeout.

## TurtleBot3 keyboard teleoperation

With the bringup running, open a second WSL terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

Use the same `ROS_DOMAIN_ID` in both terminals. Leave the bringup's
`enable_stamped_cmd_vel` at its default `true` for Jazzy teleop. Keep keyboard
focus in the teleop terminal: `w`/`x` increase/decrease forward speed,
`a`/`d` increase/decrease turning speed, and `s` or Space stops. Teleop keeps
publishing the selected speed, so releasing a key does not stop the robot.

The simulation accepts system-clock, simulation-clock, and zero-stamped velocity
commands and uses command arrival time for its watchdog, so the standard teleop
command works directly. After updating and rebuilding the driver, restart the
bringup to load the changes.

## Assets and simulation limits

The Burger URDF and STL meshes are derived from ROBOTIS `turtlebot3_description`
version 2.3.6 under Apache 2.0; see the description package's
[NOTICE](src/robot_description/NOTICE) and [LICENSE](src/robot_description/LICENSE).
The bundled arena model is a local implementation using the same meshes; see its
[NOTICE](src/robot_bringup/protos/NOTICE) and [LICENSE](src/robot_bringup/protos/LICENSE).
The default `robot_world.wbt` uses the Cyberbotics robot and furnished
environment PROTOs referenced in that file; Webots uses its cache or retrieves
missing assets. Those external assets retain their upstream licenses. Its URLs
reference Cyberbotics' moving `develop` branch, so first-launch downloads can
change over time and may produce version warnings with R2025a. Use the bundled
arena if you need to run without remote world assets.

The lidar and inertial-unit translations are aligned with the
local URDF frames. Its 20 ms simulation step can quantize sensor publication
intervals. Wheel radius is `0.033 m` and separation is
`0.160 m`. In the local arena model, collision shapes, mass distribution, and caster
contact are approximations, and the lidar supplies ideal 360-sample scans up to
3.5 m. Battery values and magnetic field are ideal placeholders in both worlds;
unmodelled bumper, cliff, sonar and button values remain zero.

Use `use_sim_time:=true` in SLAM, localization, and navigation nodes.
This bringup starts the base robot; add those applications in separate launches.
Custom `world:=/absolute/path/to/world.wbt` worlds must contain a robot named
`TurtleBot3Burger` with the device names used in the supplied PROTO and an
external controller.

The interface is based on the [ROBOTIS Jazzy driver](https://github.com/ROBOTIS-GIT/turtlebot3/blob/jazzy/turtlebot3_node/src/turtlebot3.cpp)
and [Burger bringup parameters](https://github.com/ROBOTIS-GIT/turtlebot3/blob/jazzy/turtlebot3_bringup/param/burger.yaml).

## Validation

```bash
cd ~/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select robot_bringup
colcon test-result --verbose
ros2 run robot_bringup check_bringup.py
```

The live checker requires a running simulation. Add `--test-motion` to explicitly
command a brief slow motion and verify encoder odometry responds; run it with
clear space around the robot. Unit tests exercise encoder odometry, command
message types, watchdogs, speed limits, service responses, and WSL host selection.

Development validation covered both package builds, unit tests, and live
topic/frame/timestamp, commanded turn, and command-timeout stop checks with
`robot_world.wbt` over WSL2 NAT to Windows Webots R2025a. Run the checks above to
verify your own installation.
