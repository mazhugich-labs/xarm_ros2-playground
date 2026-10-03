# Gazebo feedback through the xArm TCP emulator

## Status ownership

- This file is the working plan for connecting MoveIt's real-driver path to an xArm simulated in Gazebo.
- The user has authorized agents to mark implemented tasks complete after the relevant tests pass.
- For changes that interact with `xarm_ros2`, run integration tests against its actual packages before marking the affected task complete.
- Record test evidence and its limits; do not treat tests of the instantaneous emulator as validation of the future Gazebo backend.
- C54 injection and recovery are required acceptance cases for the emulator/driver/Gazebo integration. The overall task remains open until all required work and tests pass.

## Non-negotiable constraints

- Do not modify `xarm_ros2` or any nested submodule.
- Put all new ROS packages and emulator changes under `ros2_ws/src`.
- When creating or entering a new worktree, run `git submodule update --init --recursive` before building.
- Gazebo is the source of measured joint feedback. Commanded positions must not be reported as measured positions until Gazebo publishes them.
- Retain the real MoveIt/controller path through `uf_robot_hardware/UFRobotSystemHardware` and the xArm SDK.
- Use a separate Gazebo controller manager under `/sim/controller_manager`.
- Do not start a second MoveIt instance or a second root trajectory controller.

## Intended data flow

```text
MoveIt
  -> /xarm7_traj_controller (managed by /controller_manager)
  -> UFRobotSystemHardware and xArm SDK
  -> xArm TCP emulator
  -> /sim/joint_position_controller/commands
  -> Gazebo joints
  -> /sim/joint_state_broadcaster/joint_states
  -> xArm TCP emulator
  -> xArm SDK and driver joint states
  -> MoveIt
```

## Work plan

- [x] 1. Add the simulation controller dependency to the Docker image.
  - Install `ros-${ROS_DISTRO}-forward-command-controller` in the existing apt layer.
  - Build the complete image and confirm ROS resolves `forward_command_controller`.
  - Evidence: implemented in commit `ae6ff14`; image build and all 13 upstream xArm package builds passed.

- [x] 2. Create a standalone bringup package outside `xarm_ros2`.
  - Suggested package: `ros2_ws/src/xarm_gazebo_driver_bringup`.
  - Add package metadata, launch files, simulation controller YAML, and a wrapper Xacro.
  - Reuse installed resources from `xarm_description`, `xarm_gazebo`, and `xarm_moveit_config` without copying or editing the submodule.

- [x] 3. Define the Gazebo robot description and namespace boundaries.
  - Instantiate the upstream `xarm_device` macro for xArm7 with `gazebo_ros2_control/GazeboSystem`.
  - Configure the Gazebo plugin explicitly with namespace `/sim`.
  - Set `robot_param_node` to `/sim/robot_state_publisher`.
  - Keep simulation robot description, joint-state topics, and TF separate from the global real-driver/MoveIt graph.
  - Ensure the simulated and real descriptions use identical joint names, prefix, attachments, and mounting pose.

- [x] 4. Configure the Gazebo controller manager.
  - Configure `/sim/controller_manager` with `use_sim_time: true`.
  - Load `joint_state_broadcaster/JointStateBroadcaster` with `use_local_topics: true`.
  - Load `forward_command_controller/ForwardCommandController` for the position interfaces of `joint1` through `joint7`.
  - Use `/sim/joint_position_controller/commands` for targets.
  - Use `/sim/joint_state_broadcaster/joint_states` for measured feedback.
  - Do not load a Gazebo joint trajectory controller for this architecture.
  - Evidence: `simulation_controllers.launch.py` activates both controllers in headless Gazebo using the upstream xArm7 model. Integration tests verify simulation time, exclusive position claims, measured feedback for two targets, and namespace isolation.

- [x] 5. Add an optional Gazebo backend to `xarm_controller_emulator`.
  - Preserve the current instantaneous backend for existing tests and use cases.
  - Store commanded targets separately from measured robot state.
  - Publish accepted ServoJ targets to the Gazebo forward controller.
  - Subscribe to Gazebo joint states and map positions by joint name into the seven protocol slots, in radians.
  - Source `GET_JOINT_POS` responses and normal/rich TCP reports exclusively from measured Gazebo positions.
  - Do not wait synchronously for Gazebo while holding TCP or robot-state locks.
  - Initially reject or explicitly leave unsupported any motion command whose Gazebo behavior has not been defined; real ros2_control writes require ServoJ first.
  - Evidence: backend unit tests verify measured TCP queries/reports, input validation, and unsupported MOVE_JOINT. Headless Gazebo integration executes ServoJ through the actual `xarm_api` driver and compares TCP reads with Gazebo feedback.

- [x] 6. Define readiness, stale-feedback, stop, and fault behavior.
  - Do not report the emulator ready until one complete valid Gazebo joint-state sample has arrived.
  - Measure feedback age with a monotonic receive clock rather than comparing ROS wall time with simulation time.
  - Treat paused or lost Gazebo feedback as stale and prevent false trajectory success.
  - On stop, disable, C54, or stale feedback, hold the latest measured position and prevent queued targets from resuming on recovery.
  - Do not use command silence as the feedback watchdog: the xArm hardware plugin may omit repeated unchanged targets.
  - Evidence: unit tests cover stop/disable/mode changes, repeated timestamps, stale feedback on all wire paths, and backwards simulation time. The real-driver/Gazebo test injects C54, rejects premature clearing and motion, restores readiness, and validates pause/resume watchdog recovery. Step 9 also validates process restarts and faults during MoveIt execution.

- [x] 7. Add one launch path that starts the simulation side in dependency order.
  - Start Gazebo Classic and `/sim/robot_state_publisher`.
  - Spawn the xArm entity.
  - Wait for `/sim/controller_manager`, then spawn the broadcaster and position controller.
  - Start the emulator Gazebo backend only after its ROS interfaces are available.
  - Make readiness observable so the real-driver launch is not started against incomplete simulation feedback.
  - Evidence: installed `simulation.launch.py` passes headless startup, real-driver motion/C54/pause recovery, failed-spawn, startup-deadline, busy-port, and feedback-timeout tests. Binding errors now fail emulator startup synchronously. GUI rendering has not been tested.
  - Symlink-install regression: the readiness helper's source executable bit is required. After correcting it, all nine bringup pytest cases passed with `colcon build --symlink-install`; use this build mode for future launch validation.

- [x] 8. Connect the existing real MoveIt launch to the emulator.
  - Start the existing xArm7 real-move launch with `robot_ip:=127.0.0.1` after emulator readiness.
  - Let the real hardware plugin initialize its embedded driver; do not launch another xArm driver.
  - Keep the real-driver and MoveIt stack on wall time for the first integration while Gazebo uses simulation time.
  - Run Gazebo near real time and make pause behavior explicit in emulator readiness/fault handling.
  - Evidence: `moveit.launch.py` starts the installed real-move launch only after fresh feedback. All 11 bringup pytest cases passed with a symlink build in the Humble image, including real MoveIt plan/execute, measured Gazebo/TCP/driver/MoveIt feedback, exactly two controller managers, clock configuration, and C54 interruption/recovery without trajectory replay. Validation was headless.

- [x] 9. Validate the complete command and feedback loop.
  - Confirm exactly two controller managers: root real hardware and `/sim` Gazebo hardware.
  - Confirm controller states and claimed interfaces do not overlap.
  - Execute a small MoveIt trajectory and compare Gazebo state, emulator protocol state, driver joint state, and MoveIt execution result.
  - Verify a target is never reported as achieved before Gazebo reaches it.
  - Pause Gazebo during motion and verify feedback stops advancing and execution cannot falsely succeed.
  - Exercise C54, stop/enable recovery, stale-feedback recovery, emulator restart, and Gazebo restart.
  - Inject C54 during MoveIt execution; verify the fault is reported, motion stops, and execution cannot falsely succeed.
  - Verify clearing while the cause is active fails; releasing the cause alone leaves the error latched; clearing the error alone leaves motion disabled.
  - Restore enable/mode/readiness explicitly and execute a fresh MoveIt goal; verify rejected or pre-fault targets do not replay.
  - Run existing emulator tests and the xArm ROS 2 driver/MoveIt integration tests.
  - Final validation (2026-10-03): symlink build succeeded in the Humble/Gazebo Classic Docker image. All 17 bringup pytest cases passed, including eight combined MoveIt cases. The existing emulator suite plus actual xArm driver/MoveIt integration passed 87 cases, with one existing copyright skip. Total: 104 passed, one skipped. `colcon test-result` reports 21 bringup tests because it also counts the four CTest wrappers.
  - Acceptance evidence: both controller managers and their interface claims are checked independently. Successful MoveIt results require measured Gazebo position within the goal tolerance at completion, followed by matching driver, MoveIt, and TCP feedback. C54, stop, disable, and physics pauses during motion (including near the goal) interrupt execution without false success. Paused simulation stamps remain frozen. Explicit recovery reactivates the controller, holds the stopped position, and allows a fresh goal without replay.
  - Restart behavior: killing either the emulator or Gazebo during motion shuts down the combined launch. Tests verify launched child processes exit, TCP ports are released, and a full relaunch on the same ROS domain and ports starts at the initial pose and executes a fresh goal. Restarting an individual component under the existing driver is not supported; the upstream hardware plugin exits on TCP disconnect.
  - Limits: validation is headless and uses the packaged zero-gravity world, default xArm7, and position control. GUI rendering and physical dynamics are outside this acceptance result. `xarm_ros2` and its nested SDK remain unchanged.

## Scope note

The first integration validates the real-driver protocol and feedback loop using Gazebo position control. The existing zero-gravity world and position interface do not validate motor torque, gravity compensation, or physical fidelity. Those require a separately approved follow-up task.
