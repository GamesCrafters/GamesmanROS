# Real-arm runbook (Raspberry Pi)

You'll use 2 terminals, plus a third for emergencies. Every command is one line — paste only the command, not the code fences.

**Before you start:** the Pi needs the latest code (push from your laptop, then `git pull` on the Pi). Power on the arm, clear the area, and keep your hand near the power switch.

## Terminal 1: the container and the launch

Run these on the Pi over SSH:

```
cd ~/GamesmanROS
docker run -it --rm --name gm --net=host --ipc=host -w /ws --device=/dev/ttyAMA0 -v $HOME/GamesmanROS/ros2_ws:/ws -v $HOME/GamesmanROS/scripts:/scripts gamesmanros:humble
```

The prompt becomes `root@…:/ws#`. Rebuild only if the code changed since your last build:

```
colcon build --symlink-install
source install/setup.bash
```

Then start everything:

```
ros2 launch gamesmanros gamesmanros.launch.py game_id:=dodgem hardware:=true port:=/dev/ttyAMA0 baud:=1000000 eef_orientation:=down
```

Wait for `MycobotTopics ready`, `JointTrajectoryActionServer ready` and `ArmController ready`. Leave this terminal running.

## Terminal 2: checks and commands

```
docker exec -it gm bash
ros2 topic echo /mycobot/angles_real --once
```

**Look at the angles.** Every joint must be inside its planning limit: J1 ±165°, J2 ±132°, J3 ±147°, J4 ±142°, J5 ±162°, J6 ±177°. If one isn't, see the tip at the bottom before going on.

```
ros2 topic hz /joint_states
ros2 param set /joint_trajectory speed 20
```

`/joint_states` should print a steady rate.

**First move** (board centre, travel height):

```
ros2 action send_goal /move_arm gamesmanros_interfaces/action/MoveArm "{x: -0.05, y: 0.2125, z: 0.185}" --feedback
```

Watch the path. Cut the power if anything looks wrong. A good run ends with `success: true`.

**Then a short hop** (cell 14), and back:

```
ros2 action send_goal /move_arm gamesmanros_interfaces/action/MoveArm "{x: -0.0312, y: 0.1562, z: 0.185}" --feedback
ros2 action send_goal /move_arm gamesmanros_interfaces/action/MoveArm "{x: -0.05, y: 0.2125, z: 0.185}" --feedback
```

The numbers are metres from the base, not radians. Stay at z = 0.185 for now.

## Terminal 3 (only when needed): make the arm limp

Support the arm with one hand first, since it drops. Cut the power switch, or Ctrl-C Terminal 1 and then run this in a container shell:

```
python3 -c "from pymycobot import MyCobot; MyCobot('/dev/ttyAMA0', 1000000).release_all_servos()"
```

## Tip: stiff arm, or a joint outside its range

The arm rejects any angle outside its own range (J1 ±168°, J2 ±135°). If a joint is resting beyond that, the first point of every trajectory (the current pose) is rejected, so the arm does not move even though the log says `Executing trajectory`. Ctrl-C alone does **not** release the servos; they keep holding their last pose, which is why the arm stays stiff.

**To fix it: release the servos and restart.**

1. Ctrl-C the launch in Terminal 1, so nothing else holds the serial port.
2. Support the arm, then run the release command from the Terminal 3 section.
3. Move the arm by hand until every joint is inside its planning limit (an upright pose is a good target).
   If a joint will not move even when released, stop — that is a mechanical problem. Do not force it. Power-cycle the arm and try again.
4. Restart the launch in Terminal 1.
5. In Terminal 2, run `ros2 topic echo /mycobot/angles_real --once` and check the angles against the limits above, then retry the first move.

To lock the servos again without power-cycling, run `MyCobot('/dev/ttyAMA0', 1000000).focus_all_servos()` the same way, or send any joint command. The arm then holds wherever it is.

## If planning fails

With `2>&1 | tee ~/b.log` added to the end of the launch line, this shows the likely cause:

```
grep -n -i -E "start state|collision|bounds|IK solution|Planning failed|died" ~/b.log | tail -30
```
