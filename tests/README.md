# Tests

An automated harness that runs the demo headless and checks how the robot behaves, using Gazebo's
ground-truth pose as the reference. The recorded results are in [`results/`](results/), and the full
per-run tables are in [`results/summary.md`](results/summary.md).

## Run

After building (see the main README):

```bash
tests/run.sh demo cold1   # fresh launch, startup + the two README goals (~2 min)
tests/run.sh full full    # fresh launch, every case below (~20 min)
python3 tests/summarize.py tests/out/*   # Markdown tables
```

`run.sh` starts `demo.launch.py` with `gui:=False rviz:=False`, runs `harness.py` against it, then
stops the launch with SIGINT (like Ctrl+C) and records the shutdown. Output goes to `tests/out/<tag>/`.

## Cases

| Case | Expected |
|---|---|
| Startup and localization | Nav2 active, AMCL at the spawn pose |
| Demo goal 1, box not on the map | Succeeds; the first plan goes through the box's gap, later plans avoid it |
| Demo goal 2, box spawned while driving | Succeeds; the box lands on the current plan, later plans avoid it |
| Goal at the current pose | Succeeds immediately |
| Goal with a +90° heading | Position and heading within tolerance |
| Tour of 4 goals across the arena | All succeed |
| NavigateThroughPoses, FollowWaypoints | Succeed, no missed waypoints |
| Cancel while driving | Robot stops within 2 s and stays stopped |
| New goal while driving | Robot switches to and reaches the new goal |
| Goal inside a pillar | Observed, not judged (see findings) |
| Goal outside the walls / off the map | Aborts |
| Goal boxed in by spawned boxes | Aborts after recoveries; robot stays outside |
| Same goal after the boxes are removed | Succeeds |
| Goal sent on `/goal_pose` (what RViz's 2D Goal Pose does) | Robot reaches it and stops |

Every case also records the largest commanded speeds, the closest lidar return, collision monitor
activity, ground-truth roll and pitch, and the AMCL error against ground truth. A case fails if the
speed limits (0.26 m/s, 1.82 rad/s) are exceeded or roll or pitch goes above 0.2 rad.

## Results

From 7 fresh launches: 1 full suite and 5 cold-start demo runs, all with the scripts in this folder,
plus one launch where Nav2 never started (below).

- **Demo goals:** both succeeded in 6 of 6 runs, with 0 recoveries. Goal 1 took 31.4–33.3 s and
  rerouted around the box 9.2–10.3 s after the goal was sent. Goal 2 took 32.4–34.4 s. In every
  run, both boxes were on the plan before the robot saw them and off it afterwards.
- **Full suite:** 18 PASS, 1 INFO (the pillar goal).
- **Limits:** commanded speed never went above 0.26 m/s or 1.80 rad/s. The largest roll or pitch
  was 0.006 rad, and the largest AMCL error was 0.22 m.
- **Startup:** Nav2 was active 5.6–7.3 s after launch.
- **Shutdown:** SIGINT stopped everything in 1.6–3.3 s, with no processes left behind.

## Findings

1. **A goal inside an obstacle reports success.** The goal inside the pillar at (0, 0) ended
   `SUCCEEDED` 0.55 m from the requested point. NavFn's `tolerance: 0.5` moves the path end to free
   space, and the goal checker checks against that path end. Setting the tolerance to 0 would make
   such goals abort, but RViz clicks close to obstacles would then fail too. **Kept at Nav2's default
   of 0.5 on purpose:** the task is reaching goals in free space while avoiding obstacles, and a
   forgiving goal makes hand-clicked RViz goals near walls work.
2. **Nav2 startup hung in 1 of 7 launches.** `route_server`'s reply to the lifecycle manager's
   configure request was lost (`failed to send response to /route_server/change_state (timeout)`),
   and `bt_navigator` stayed `unconfigured`. Gazebo reported `Network is unreachable` at the same
   time, so a network change during startup is the likely trigger. See
   [`results/hung_startup_1/`](results/hung_startup_1/). The harness now fails after 120 s instead
   of waiting forever.
3. **`collision_monitor` segfaults on shutdown in 2 of 6 runs.** This is upstream, after navigation
   has stopped. The launch still exits with code 0 and leaves no processes behind.
4. **A boxed-in goal takes about 5 minutes to give up** (296 s and 25 recoveries in the recorded
   run). That run used a 300 s harness limit; it is now 420 s.

## How the results are scored

- Positions are checked against Gazebo ground truth with 5 cm added to Nav2's 0.25 m goal
  tolerance. Nav2 decides it has arrived using its AMCL estimate, which is a few cm off.
- The closest lidar return is not proof of zero contact. The lidar sits off-centre on a rectangular
  robot.
- The removed-boxes case follows an abort whose recoveries clear the costmaps, so its pass doesn't
  show that lidar raytracing alone cleared them.
- `harness.py` doesn't use `BasicNavigator.waitUntilNav2Active()`. It never times out, and it
  publishes its own initial pose at the map origin, which would reset AMCL onto the centre pillar.
- Not covered: the Gazebo and RViz windows themselves, obstacles that keep moving, a kidnapped
  robot, and real hardware.
