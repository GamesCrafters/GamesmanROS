# Real-arm bring-up

How a planned move reaches the real MyCobot (all ported from the ROS 1 system in
`catkin_ws/`):

```
arm_controller (MoveItPy: LIN, then OMPL)
   └─ execute ─▶ joint_trajectory ──▶ /mycobot/angles_goal ─▶ mycobot_topics ─▶ serial ─▶ arm
                                                                       ▲
 /joint_states ◀── joint_state_bridge ◀── /mycobot/angles_real ◀──────┘
 gripper_controller ──▶ /mycobot/gripper_status ─▶ mycobot_topics
```

| New piece | Ported from (ROS 1) |
|---|---|
| `mycobot_interfaces` (messages) | `mycobot_communication/msg` |
| `mycobot_topics` | `mycobot_communication/scripts/mycobot_topics.py` |
| `joint_trajectory` | `gamesmanros/src/joint_trajectory.py` |

**Run everything on the one machine the arm is plugged into** (the Pi, or a Linux
laptop). DDS between Docker on Windows/macOS and another machine is unreliable
(see `SETUP.md`).

## Before touching the arm

1. **Rebuild** (new package + new dependencies): `colcon build --symlink-install`, then
   `source install/setup.bash`. The Dockerfile now installs `pymycobot`; in an
   old container run `pip3 install pymycobot`.
2. **Rehearse on the mock first.** With the demo up (`demo.launch.py`) and
   `gamesmanros.launch.py game_id:=dodgem`, a full move should now complete,
   gripper included (one line — do not wrap it):
   ```
   ros2 action send_goal /execute_move gamesmanros_interfaces/action/ExecuteMove "{game_id: dodgem, move_string: M_12_13_x, current_position: a, new_position: b}" --feedback
   ```
3. **Run everything on the Pi, not on your laptop.** Nothing on the laptop changes;
   use it only to SSH in (and to commit/push this branch so the Pi can pull it). On
   the Pi, with Docker installed:
   ```
   git pull
   docker build -f docker/Dockerfile -t gamesmanros:humble docker/
   docker run -it --rm --net=host --ipc=host -w /ws \
       --device=/dev/ttyUSB0 -v "$PWD/ros2_ws:/ws" gamesmanros:humble
   ```
   `--device` hands the **Pi's** serial port to the container. Use `/dev/ttyAMA0` for
   a Pi-mounted arm. Pass it on the command line like this — do not uncomment
   `devices:` in `docker/docker-compose.yml`, which is shared with laptops. You may
   need to add your user to the `dialout` group, and for `/dev/ttyAMA0` enable the
   Pi's serial port (and turn off its login console) first.

## At the robot (hand on the power switch)

Ports/baud: USB arm `/dev/ttyUSB0` @ 115200; Pi-mounted arm `/dev/ttyAMA0` @ 1000000.

| # | Step | Pass if |
|---|---|---|
| 1 | `ros2 run gamesmanros mycobot_topics --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200` and `ros2 topic echo /mycobot/angles_real --once` | six plausible angles, in degrees |
| 2 | add `ros2 run gamesmanros joint_state_bridge`; open RViz via `demo.launch.py hardware:=true` | the model in RViz matches the real arm's pose (ROS 1 used this same plain degrees→radians mapping) |
| 3 | `ros2 action send_goal /move_gripper gamesmanros_interfaces/action/MoveGripper "{open: true}"` then `false` | gripper opens, then closes |
| 4 | Stop 1–3. Start: `ros2 launch mycobot_280_moveit_config demo.launch.py hardware:=true`, then `ros2 launch gamesmanros gamesmanros.launch.py game_id:=dodgem hardware:=true port:=/dev/ttyUSB0 baud:=115200` | logs show `MycobotTopics ready`, `JointTrajectoryActionServer ready`, `ArmController ready` |
| 5 | **Slow down first:** `ros2 param set /joint_trajectory speed 20` | (default is 40, the ROS 1 value) |
| 6 | One move to hover above the board centre: `ros2 action send_goal /move_arm gamesmanros_interfaces/action/MoveArm "{x: -0.05, y: 0.2125, z: 0.185}" --feedback` | arm moves there, result `success: true` |
| 7 | `ros2 run gamesmanros reach_sweep --ros-args -p game:=dodgem -p mode:=cells -p settle:=1.0` | same reachable cells as the mock sweep |
| 8 | One full move: the `/execute_move` command from step 2 above | piece picked and placed |
| 9 | `ros2 run gamesmanros game_manager` | a game |

## Know before you go

* **Gripper orientation.** `arm_controller` defaults to the rotated+tilted gripper
  pose; the ROS 1 controller dropped it because of IK failures on the outer rows.
  Compare sweeps with `eef_orientation:=down` (launch argument) and pick one.
* **Board calibration.** Cell positions use the ROS 1 constants unchanged
  (`robot_control.py`). Check one cell by eye before running a game.
* **Execution is not verified.** `arm_controller` does not check MoveIt's execution
  result, and `joint_trajectory` only *warns* if the arm does not reach the final
  point within `settle_timeout` (6 s). A stalled arm will not stop the game.
* **Joint states arrive at ~4 Hz** (the ROS 1 driver's poll rate), so
  `allowed_start_tolerance` was loosened from 0.01 to 0.05 rad.
* MoveIt's trajectory-duration monitor is **off** (`execution_duration_monitoring`),
  because the executor waits for the arm to arrive; it bounds that wait itself.
* **Gripper actuation** is a fixed 1.5 s wait, as in ROS 1.
* **No failure recovery yet.** A failed move leaves the game state wrong; see
  `docs/FAILURE_RECOVERY.md` once that lands.
* Joint position limits are still ±π in the URDF; tighten to the manufacturer's values.
