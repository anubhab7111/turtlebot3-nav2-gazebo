# TurtleBot3 Autonomous Navigation with Nav2 in Gazebo

A TurtleBot3 Waffle navigates to goals with ROS 2 Nav2 in Gazebo. It avoids the obstacles on its
map, and also boxes that are **not** on the map: it detects them with its lidar while driving and
replans around them.

Tested on Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Nav2 1.3.13.

## Setup

Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
(`ros-jazzy-desktop`, `ros-dev-tools`), then:

```bash
source /opt/ros/jazzy/setup.bash
git clone https://github.com/anubhab7111/turtlebot3-nav2-gazebo.git
cd turtlebot3-nav2-gazebo
sudo rosdep init  # skip if rosdep is already initialized
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

## Run

```bash
source install/setup.bash
ros2 launch tb3_nav2_demo demo.launch.py
```

Gazebo and RViz open with the robot already localized.

1. **Goal 1: a box the robot doesn't know about.** The orange box at (1.1, 0.55) is in the Gazebo
   world but **not on the robot's map**, so the first plan goes straight through it. In RViz, click
   **2D Goal Pose** and click-drag a goal on the right side of the arena, near (1.8, 0.5).
   **Watch for the replan around the orange box about 9–10 s after you send the goal**, when the
   lidar sees it: the red planned path jumps to another gap between the pillars.
2. **Goal 2: a box dropped in while driving.** Send a goal back to the start, near (-2.0, -0.5).
   While the robot drives along the lower corridor, drop a box in its way from a second terminal.
   The robot replans as soon as its lidar sees the new box:

   ```bash
   source install/setup.bash
   ros2 run ros_gz_sim create -world default -name live_obstacle \
     -file $(ros2 pkg prefix --share tb3_nav2_demo)/models/live_obstacle.sdf -x -1.1 -y -0.55
   ```

Options: `gui:=False` (no Gazebo window), `rviz:=False` (no RViz).

## What is configured

The `tb3_nav2_demo` package wraps Nav2's official TurtleBot3 simulation and changes only this:

- **`config/nav2_params.yaml`**: Nav2's defaults, plus:
  - an automatic AMCL initial pose,
  - the real Waffle speed limits (0.26 m/s, 1.82 rad/s),
  - a rotation shim that turns the robot towards its path before MPPI takes over.
- **`worlds/obstacles_world.sdf.xacro`**: the sandbox world plus the unmapped box.
- **`models/live_obstacle.sdf`**: the box dropped in while the robot drives.
- **`launch/demo.launch.py`**: starts everything with one command. It runs Nav2 as separate
  processes and starts the Gazebo GUI and RViz itself, so Ctrl+C stops everything in about 2 s and
  leaves no processes behind. Nav2's `collision_monitor` sometimes segfaults while shutting down;
  this happens after navigation has stopped and is an upstream issue.
- **`rviz/nav2_demo.rviz`**: Nav2's default view without the Nav2 panels, which crash RViz on exit in
  Jazzy.

## Tests

[`tests/`](tests/) has a harness that runs the demo headless and checks 19 cases against Gazebo's
ground-truth pose: both demo goals (including whether the robot really rerouted), a tour, waypoints,
cancel, preemption, unreachable and boxed-in goals, and goals sent like RViz's 2D Goal Pose. Over 6
fresh launches, both demo goals succeeded every time with 0 recoveries. Results and findings are in
[`tests/README.md`](tests/README.md).

```bash
tests/run.sh demo cold1   # ~2 min
tests/run.sh full full    # ~20 min
```

## Challenges and learnings

Most of the work was not navigation itself but getting the stack to start, stop and behave the same
way every time, and then proving it.

- **Nav2 aborted at startup.** With the stock launch, nothing navigated until someone clicked
  "2D Pose Estimate": AMCL had no initial pose, so the `map -> odom` transform never appeared, and
  Nav2's lifecycle manager gave up activating the costmaps. Fix: AMCL sets the spawn pose itself
  (`set_initial_pose`). *Learning: Nav2's bringup depends on a complete TF tree.*
- **The Gazebo GUI and RViz silently never started.** My `headless`/`use_rviz` arguments were
  overwritten by the values I passed to Nav2's included launch file, because launch arguments are
  not scoped to an include. Fix: my own names, `gui` and `rviz`. *Learning: in ROS 2 launch,
  arguments passed to an include become global.*
- **Goal 1 stalled at the start in 3 of 5 runs.** The robot starts facing east, but its path heads
  north. MPPI crept through the turn next to a wall, tripped the progress checker (0.5 m in 10 s) and
  started recovery behaviours. Fix: a rotation shim turns the robot to face its path first. After
  that, goal 1 ran clean in 6 of 6 test launches. *Learning: read which check failed before tuning
  the controller.*
- **Ctrl+C left a mess.** It left an orphaned Gazebo window, a Nav2 container that hung until it
  was killed, and an RViz crash. Fixes: start the Gazebo GUI directly instead of through a shell,
  run Nav2 as separate processes, and drop the Nav2 RViz panels. The panels were found by bisection:
  each one alone crashes RViz on exit. One upstream issue is left: `collision_monitor` segfaults on
  shutdown in 2 of 6 recorded runs, with no processes left behind. *Learning: ROS 2 launch only
  signals its direct children.*
- **"Goal succeeded" doesn't prove obstacle avoidance.** So the test harness records every plan and
  checks that it went through each box before the lidar could see it and avoided it afterwards.
  Building it caught two traps: Nav2's `waitUntilNav2Active()` helper quietly resets AMCL to the map
  origin, and one of my own test runs spawned a box on top of the robot and still "passed".
  *Learning: check a test's preconditions, not just its result.*
- **Nav2 startup hung in 1 of 7 launches.** `route_server`'s reply to the lifecycle manager was lost
  during a network drop, so `bt_navigator` never left `unconfigured` (log in
  [`tests/results/hung_startup_1/`](tests/results/hung_startup_1/)). My harness had no deadline for
  startup and waited 2 hours. It now fails after 120 s and records the failure. *Learning: bound
  every wait, and keep the failures as evidence.*

## Credits

The world, parameters and RViz view are derived from
[Nav2](https://github.com/ros-navigation/navigation2).
