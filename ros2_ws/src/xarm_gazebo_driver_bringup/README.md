# xArm Gazebo real-driver bringup

This package contains the simulation description and controller configuration for
an unprefixed xArm7 without a gripper, using ROS 2 Humble and Gazebo Classic 11.
It reuses the installed `xarm_description` macros and meshes. The wrapper uses the
upstream default mounting transform (world origin, no rotation), matching the
default real-driver description.

The package provides separate launch files for the simulation description and
controller activation. The emulator now supplies an optional Gazebo backend.
Automatic Gazebo spawning and the combined MoveIt launch are subsequent tasks in the repository's
`GAZEBO_REAL_DRIVER_TODO.md`.

## Build and inspect

Inside the project Docker environment:

```bash
source /opt/ros/humble/setup.bash
source /opt/xarm_ros2_ws/install/setup.bash
cd /opt/ros2_ws
colcon build --packages-up-to xarm_gazebo_driver_bringup
source install/setup.bash
ros2 launch xarm_gazebo_driver_bringup simulation_description.launch.py
```

The launch creates `/sim/robot_state_publisher` with simulation time enabled. It
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
together. A later spawn launch must preserve this mounting convention.

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

The four pytest cases passed in the Humble Docker image, including spawning,
controller activation, and measured joint feedback in Gazebo Classic 11.
The backend integration case also starts the TCP emulator with `backend:=gazebo`
and the real `xarm_api` driver against loopback. It verifies ServoJ commands and
measured position queries, C54 latching/clearing/recovery, physics-pause watchdog
behavior, and explicit recovery without replaying rejected targets. Run these
tests in an isolated container: they own TCP ports 502, 30001, and 30002.
MoveIt trajectory execution with Gazebo remains a separate integration task.

## Resources for subsequent launch integration

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
The root controller manager and MoveIt will belong to the real-driver stack.
The simulation configuration must not supply global joint states or a second
trajectory controller.

The plugin configuration follows the Humble
[gazebo_ros2_control documentation](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html).
The position controller follows the
[forward command controller documentation](https://control.ros.org/humble/doc/ros2_controllers/forward_command_controller/doc/userdoc.html).
