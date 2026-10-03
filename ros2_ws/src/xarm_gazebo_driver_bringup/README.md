# xArm Gazebo real-driver bringup

This package contains the simulation description and controller configuration for
an unprefixed xArm7 without a gripper, using ROS 2 Humble and Gazebo Classic 11.
It reuses the installed `xarm_description` macros and meshes. The wrapper uses the
upstream default mounting transform (world origin, no rotation), matching the
default real-driver description.

The package starts Gazebo, the robot, simulation controllers, and the emulator
through one ordered launch. `moveit.launch.py` then starts the upstream real-driver
MoveIt stack once fresh Gazebo feedback is available.

## Build and start the simulation

Inside the project Docker environment:

```bash
source /opt/ros/humble/setup.bash
source /opt/xarm_ros2_ws/install/setup.bash
cd /opt/ros2_ws
colcon build --symlink-install --packages-up-to xarm_gazebo_driver_bringup
source install/setup.bash
ros2 launch xarm_gazebo_driver_bringup simulation.launch.py gui:=false
```

Omit `gui:=false` to show Gazebo. The default world has a ground plane, zero
gravity, and a nominal real-time factor of 1. The xArm is spawned at the world
origin, preserving its description's mounting transform. No external models
need downloading for this world.

Startup advances on successful process exits: robot spawn, controller
activation, then emulator startup. The emulator starts with `backend:=gazebo`.
The launch prints `SIMULATION_READY` only after receiving fresh emulator
feedback. `/xarm_controller_emulator/feedback_ready` continues to report feedback
availability after startup. This signal does not mean motors are enabled or
that C54 is clear; the driver still performs its normal readiness sequence.

Arguments:

| Argument | Default | Purpose |
| --- | --- | --- |
| `gui` | `true` | Show the Gazebo client. |
| `world` | Packaged `worlds/xarm.world` | Choose a Gazebo Classic world file. |
| `startup_timeout` | `60.0` seconds | Bound simulation startup through emulator feedback readiness. |
| `feedback_timeout` | `0.5` seconds | Emulator's stale-feedback threshold. |
| `launch_moveit` | `false` | Start real-driver MoveIt after readiness; enabled by `moveit.launch.py`. |
| `show_rviz` | `true` | Show RViz when MoveIt is enabled. |

Failed spawning, controller activation, or emulator startup stops the launch.
Occupied emulator TCP ports are startup errors. Gazebo server exit also shuts
down the launch. Ctrl+C stops the launched processes. A later physics pause
does not stop the launch: the emulator watchdog handles loss of feedback.

The simulation launch does not start MoveIt or the real driver. A subsequent
launcher can wait for current feedback using the installed readiness helper:

```bash
ros2 run xarm_gazebo_driver_bringup wait_for_feedback --timeout 30
```

It exits zero on fresh feedback and nonzero on timeout.

## Start Gazebo with real-driver MoveIt

After building and sourcing the overlay above, start the combined launch instead
of the simulation-only launch:

```bash
ros2 launch xarm_gazebo_driver_bringup moveit.launch.py
```

For headless operation:

```bash
ros2 launch xarm_gazebo_driver_bringup moveit.launch.py gui:=false show_rviz:=false
```

The launch includes the installed `xarm7_moveit_realmove.launch.py` with
`robot_ip:=127.0.0.1`. Its `UFRobotSystemHardware` plugin owns the embedded driver
and the root `xarm7_traj_controller`; no separate driver is needed. The robot
configuration is the same unprefixed xArm7 without a gripper at the world origin.
Do not run another simulation, driver, or MoveIt launch alongside it.

MoveIt and the real controller manager use wall time. Gazebo's controller manager
uses simulation time, with the packaged world running nominally at real time.
Pausing physics beyond `feedback_timeout` makes the emulator stop accepting
motion; resuming physics alone does not re-enable it. `SIMULATION_READY` reports
simulation readiness, not completion of MoveIt's subsequent startup.

### C54 injection and recovery in the emulator

Inject C54 using the emulator service:

```bash
ros2 service call /xarm_controller_emulator/set_c54 std_srvs/srv/SetBool '{data: true}'
```

An active MoveIt execution fails and the real driver deactivates its trajectory
controller. Clearing the error while the injected cause remains active fails.
Release the cause, clear the latched error, and explicitly restore readiness:

```bash
ros2 service call /xarm_controller_emulator/set_c54 std_srvs/srv/SetBool '{data: false}'
ros2 service call /xarm/clean_error xarm_msgs/srv/Call '{}'
ros2 service call /xarm/motion_enable xarm_msgs/srv/SetInt16ById '{id: 8, data: 1}'
ros2 service call /xarm/set_mode xarm_msgs/srv/SetInt16 '{data: 1}'
ros2 service call /xarm/set_state xarm_msgs/srv/SetInt16 '{data: 0}'
```

The driver reactivates its trajectory controller once ready. Submit a fresh
MoveIt goal; the interrupted trajectory is not resumed. These are emulator fault
injection semantics, not instructions for diagnosing C54 on physical hardware.

## Individual components

For manual composition, `simulation_description.launch.py` creates
`/sim/robot_state_publisher` with simulation time enabled. It
publishes `/sim/robot_description`, `/sim/tf`, and `/sim/tf_static`, and consumes
`/sim/joint_state_broadcaster/joint_states`. Dynamic transforms require joint
feedback; the launch itself does not generate joint states or a simulation clock.

After spawning this description in Gazebo, activate its controllers with:

```bash
ros2 launch xarm_gazebo_driver_bringup simulation_controllers.launch.py
```

The spawner waits up to 30 seconds for `/sim/controller_manager`, then loads and
activates the joint-state broadcaster and position forward controller. It exits
after activation; both controllers remain in the Gazebo process. This launch
does not spawn the robot or start the emulator.

## Description compatibility and namespace validation

The current supported model is the upstream default xArm7: empty joint prefix,
no gripper or other attachments, and `world_joint` at `xyz="0 0 0"`, `rpy="0 0 0"`.
Link and joint names remain identical to the real-driver description; only the
simulation ROS topics and nodes use `/sim`. Adding a prefix, tool, or non-default
mounting pose requires updating both descriptions and the controller joint list
together. The coordinated spawn launch uses this same mounting convention.

The tests compare the expanded simulation and real-driver descriptions, including
geometry, inertias, joint limits, mounting transform, and hardware interfaces.
A headless integration test spawns the robot in a private Gazebo world while a
global description publisher also exists. It verifies that Gazebo obtains the
simulation model, exposes its interfaces through `/sim/controller_manager`, and
keeps simulation description and TF publishers out of global topics.
It also activates both controllers, verifies their interface claims and
simulation-time configuration, and sends two joint targets whose measured
positions must arrive through the Gazebo joint-state broadcaster.

After building and sourcing the overlay, run in an isolated container or an
unused ROS domain (83 is an example):

```bash
ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 colcon test \
  --packages-select xarm_gazebo_driver_bringup --event-handlers console_direct+
colcon test-result --verbose
```

All 11 bringup pytest cases passed with a symlink build in the Humble Docker image,
including spawning,
controller activation, and measured joint feedback in Gazebo Classic 11.
The backend integration case also starts the TCP emulator with `backend:=gazebo`
and the real `xarm_api` driver against loopback. It verifies ServoJ commands and
measured position queries, C54 latching/clearing/recovery, physics-pause watchdog
behavior, and explicit recovery without replaying rejected targets. Run these
tests in an isolated container: they own TCP ports 502, 30001, and 30002.
The combined-launch tests execute MoveIt trajectories through the real hardware
plugin and compare Gazebo, driver, MoveIt, and TCP joint feedback. They verify
startup ordering, controller-manager isolation, clock configuration, C54 during
motion, explicit recovery, and a fresh successful goal without trajectory replay.
Process restarts and pausing during MoveIt execution remain step 9 acceptance work.
The coordinated-launch tests repeat driver motion, C54 recovery, and pause/resume
through the installed launch. They also check failed spawning, a startup deadline,
occupied emulator TCP ports, and readiness timeout without feedback. Validation
was headless; GUI rendering was not tested.

Run launch tests with `--symlink-install` as above: installed script symlinks
retain source permissions. The `scripts/wait_for_feedback` helper must be tracked
as executable; a regular copy install can hide a missing source executable bit.

## Resources

- `urdf/xarm7_sim.urdf.xacro`: upstream xArm7 model with `GazeboSystem` hardware
  and one explicitly namespaced Gazebo ros2_control plugin.
- `config/sim_controllers.yaml`: `/sim/controller_manager` configuration for a
  joint-state broadcaster and a position forward controller.
- Gazebo targets: `/sim/joint_position_controller/commands`
  (`std_msgs/msg/Float64MultiArray`, joint1 through joint7, radians).
- Gazebo measurements: `/sim/joint_state_broadcaster/joint_states`
  (`sensor_msgs/msg/JointState`).

The Gazebo plugin creates its controller manager when the model is spawned.
Publishing this description alone does not create or activate controllers.
The root controller manager and MoveIt belong to the real-driver stack.
The simulation configuration must not supply global joint states or a second
trajectory controller.

The plugin configuration follows the Humble
[gazebo_ros2_control documentation](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html).
The position controller follows the
[forward command controller documentation](https://control.ros.org/humble/doc/ros2_controllers/forward_command_controller/doc/userdoc.html).
