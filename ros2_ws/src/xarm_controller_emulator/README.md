# xArm controller emulator

A small ROS 2 emulator for the xArm private Modbus-TCP protocol. It listens on
loopback: control on port 502, normal reports on 30001, and rich reports on 30002.
Normal and rich reports run at 5 Hz. Port 502 may require permission to bind a
privileged port (the project's Docker environment runs the emulator as root).
All three endpoints bind before the ROS event loop starts; a port conflict fails
the process instead of leaving an apparently ready emulator with a dead server.

The wire format follows the [xArm Developer Manual V2.0.1, section 2.1](https://www.ufactory.cc/wp-content/uploads/2026/04/xArm-Developer-Manual-V2.0.1.pdf):
header/report integers are big-endian and float32 fields are little-endian.
Private control protocol IDs 2 and 3 are accepted and echoed in replies. The
bundled C++ SDK switches to ID 3 for heartbeat-enabled control after reading a
firmware version >= 1.8.6; rejecting it disconnects the real ROS driver.
Supported registers are `01`, `0B`–`11`, `13`, `17`, `1D`, `29`, `2A`, and `6A`
(hexadecimal). Replies carry error, warning and readiness flags. Unsupported
commands, malformed parameters, and unsupported modes return the SDK's invalid
request bit (`0x08`). Invalid protocol IDs and empty frames close the connection.

## Motion and recovery

With the default `instantaneous` backend, the initial controller is enabled and
idle (reported state 2), in position mode.
Only modes 0 (position), 1 (ServoJ), and 2 (joint teaching) are accepted. Joint
motion requires mode 0; ServoJ requires mode 1. Both require enabled motors,
readiness, and no active controller error. Warnings do not block motion.

Motion-enable, mode changes, and error clearing enter reset state 5 and clear the
command buffer. Send `SET_STATE(0)` to become ready again; the report then shows
idle state 2. State 3 pauses motion and state 4 stops it. A stopped or paused
emulator never changes joint positions in response to motion commands.

`~/set_c54` (`std_srvs/srv/SetBool`) injects a simulated fault. Setting it true
latches error 54, stops motion, clears commands, and returns to mode 0. Setting
it false releases the condition but leaves the error latched. Recovery requires
releasing the condition, `CLEAN_ERR`, enabling motors if needed, then
restoring the intended mode and `SET_STATE(0)`. This C54 latch policy is an emulator convention, not a hardware
behavior verified by the cited manual.

## Gazebo feedback backend

After spawning the unprefixed xArm7 model and activating the controllers from
`xarm_gazebo_driver_bringup`, start:

```bash
ros2 run xarm_controller_emulator xarm_controller_emulator \
  --ros-args -p backend:=gazebo -p feedback_timeout:=0.5
```

Alternatively, `ros2 launch xarm_gazebo_driver_bringup simulation.launch.py`
starts Gazebo, the robot, controllers, and this backend in order, with a feedback
readiness gate. Do not start a second emulator alongside that launch.

The backend streams ServoJ targets to `/sim/joint_position_controller/commands`
and consumes `/sim/joint_state_broadcaster/joint_states`. It maps feedback by joint
name, validates complete finite measurements, and uses measured positions for
both joint queries and TCP reports. Accepting a command never changes measured
positions. `MOVE_JOINT` is rejected in this backend because no joint trajectory
planner is implemented. The default instantaneous backend is unchanged.

The emulator starts stopped and publishes `~/feedback_ready` (`std_msgs/msg/Bool`).
Wait for true before starting the real driver, which must still set mode and
readiness. This topic indicates fresh feedback, not motor readiness or absence
of C54. No joint targets are published before the first valid measurement.

Feedback freshness uses monotonic receipt time and requires advancing joint-state
timestamps. Repeated timestamps do not keep the backend ready. A steady-clock
watchdog remains active while Gazebo is paused. Missing feedback stops the
controller (state 4, mode 0), discards targets, and holds the last measurement;
it does not invent a manufacturer error code. Fresh feedback alone does not
restore motion. Restore ServoJ mode and readiness explicitly after resuming
Gazebo. A backwards simulation timestamp also stops motion and discards targets.
Stop, pause, disable, mode changes, and C54 similarly replace targets with a hold.

This backend currently supports the bringup package's fixed seven-joint, empty
prefix configuration. The controller path remains on wall time; feedback stamps
are used only to detect simulation progress and resets, not compared to wall time.

## Intentional limits

- With the instantaneous backend, motion completes immediately. There is no trajectory planner, timing, queue,
  collision detection, or joint-limit validation. The completed `MOVE_JOINT`
  response includes a command-buffer count of zero.
- Nonzero `only_check_type` is rejected without moving: this emulator cannot
  truthfully validate a path. An optional zero check byte is accepted.
- TCP pose and joint torques are static placeholders; there is no forward
  kinematics or dynamics model.
- Rich reports use the SDK-compatible 245-byte format. Later fields described
  by the manual (temperatures, velocities, GPIO, force sensor, etc.) and the
  developer report endpoint on port 30003 are not implemented.
- IO, Cartesian motion, velocity modes, brake commands, and other unlisted
  registers are not implemented. The advertised firmware string is a client
  compatibility identifier, not a claim of full firmware emulation.

## Validation

In a sourced ROS 2 environment, from this package directory:

```bash
PYTHONPATH=. python3 -m pytest -q
```

The tests use independent wire encoders/decoders, stream sockets, concurrency
checks, and a real ROS service client. They cover endian conversion, fixed field
offsets, frame fragmentation, invalid requests, motion interlocks, reset effects,
C54 recovery, and report streams. The generated copyright test remains skipped.

### Real driver and MoveIt integration

The opt-in suite launches `xarm_api/xarm7_driver.launch.py` with both normal and
rich reporting, then `xarm_moveit_config/xarm7_moveit_realmove.launch.py` with
RViz disabled. It verifies SDK protocol switching, repeated driver service
calls, joint-state reports, an active trajectory controller, and two successful
MoveGroup plan-and-execute goals (joint 1 to 0.15 rad and back). It also injects
C54 through ROS and verifies blocked motion, fault latching, explicit recovery,
and successful fresh commands. Execution uses
`UFRobotSystemHardware` and the emulator TCP sockets, not fake hardware.

Run in an isolated container so tests can own ports 502/30001/30002 and the ROS
graph. From this package directory, with the project's built Docker image:

```bash
docker run --rm --network none \
  -e PYTHONDONTWRITEBYTECODE=1 -e XARM_ROS2_INTEGRATION=1 \
  -v "$PWD:/package:ro" -w /package \
  --entrypoint bash xarm-ros2-playground:latest -lc '
    source /opt/ros/humble/setup.bash
    source /opt/xarm_ros2_ws/install/setup.bash
    export PYTHONPATH=/package:$PYTHONPATH
    python3 -m pytest -q -p no:cacheprovider
  '
```

The tests are skipped unless `XARM_ROS2_INTEGRATION=1`. To run them in an already
isolated, sourced environment, use
`XARM_ROS2_INTEGRATION=1 python3 -m pytest -q integration/test_xarm_ros2.py`.
Failure output includes the launched processes' logs; pytest's temporary
directory also retains each emulator, driver, and MoveIt log.

The `xarm_gazebo_driver_bringup` test suite separately runs the Gazebo backend
against the real `xarm_api` driver and upstream robot model. It checks measured
joint feedback, C54 injection/recovery, and stopped motion during a physics
pause, followed by explicit recovery without replaying rejected targets.
MoveIt trajectory execution with Gazebo remains a later integration step.
