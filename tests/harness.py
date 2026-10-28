#!/usr/bin/env python3
"""Nav2 TurtleBot3 test harness. Usage: harness.py full|demo <results.json>

Needs a running `demo.launch.py` (run.sh starts one). Ground truth comes from Gazebo (`gz model -p`).
demo: startup + the two README goals. full: demo + every other case below.
"""
import json, math, os, re, subprocess, sys, threading, time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import CollisionMonitorState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf2_ros

SDF = os.path.join(get_package_share_directory('tb3_nav2_demo'), 'models', 'live_obstacle.sdf')
BOX1 = (1.1, 0.55)       # unmapped static box in the world
LIVE = (-1.1, -0.55)     # live box spawn point
VX_MAX, WZ_MAX, EPS = 0.26, 1.82, 1e-3
XY_TOL, YAW_TOL = 0.25, 0.25


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Monitor(Node):
    def __init__(self):
        super().__init__('test_monitor')
        q = qos_profile_sensor_data
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, q)
        self.create_subscription(Odometry, 'odom', self.on_odom, q)
        self.create_subscription(LaserScan, 'scan', self.on_scan, q)
        self.create_subscription(Path, 'plan', self.on_plan, q)
        self.create_subscription(CollisionMonitorState, 'collision_monitor_state', self.on_cm, q)
        self.create_subscription(Clock, 'clock', self.on_clock, q)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.tf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tf, self)
        self.lock = threading.Lock()
        self.odom_v = (0.0, 0.0)
        self.sim_t = 0.0
        self.reset()

    def reset(self):
        with self.lock:
            self.max_vx = 0.0; self.max_wz = 0.0; self.min_scan = float('inf')
            self.cm_triggers = set(); self.plans = []

    def on_cmd(self, m):
        with self.lock:
            self.max_vx = max(self.max_vx, abs(m.linear.x)); self.max_wz = max(self.max_wz, abs(m.angular.z))

    def on_odom(self, m):
        self.odom_v = (m.twist.twist.linear.x, m.twist.twist.angular.z)

    def on_scan(self, m):
        r = [x for x in m.ranges if m.range_min <= x <= m.range_max]
        if r:
            with self.lock:
                self.min_scan = min(self.min_scan, min(r))

    def on_cm(self, m):
        if m.action_type != 0:
            with self.lock:
                self.cm_triggers.add((m.action_type, m.polygon_name))

    def on_clock(self, m):
        self.sim_t = m.clock.sec + m.clock.nanosec * 1e-9

    def on_plan(self, m):
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses[::3]]
        with self.lock:
            self.plans.append((time.time(), self.pose(), pts))

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_footprint', Time())
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (t.transform.translation.x, t.transform.translation.y, yaw)
        except Exception:
            return None


def gt_pose():
    out = subprocess.run(['gz', 'model', '-m', 'turtlebot3_waffle', '-p'], capture_output=True, text=True, timeout=20).stdout
    nums = re.findall(r'\[([-\d.e ]+)\]', out.split('Pose')[-1])
    xyz = [float(v) for v in nums[0].split()]; rpy = [float(v) for v in nums[1].split()]
    return xyz, rpy


def spawn(name, x, y):
    r = subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'default', '-name', name,
                        '-file', SDF, '-x', str(x), '-y', str(y)], capture_output=True, text=True, timeout=30)
    return 'successful' in (r.stdout + r.stderr)


def remove(name):
    r = subprocess.run(['gz', 'service', '-s', '/world/default/remove', '--reqtype', 'gz.msgs.Entity',
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
                        '--req', f'name: "{name}" type: MODEL'], capture_output=True, text=True, timeout=20)
    return 'true' in r.stdout


def wait_active(node, name, timeout):
    """Poll <name>/get_state until 'active'. Bounded, unlike BasicNavigator.waitUntilNav2Active."""
    cli = node.create_client(GetState, f'{name}/get_state')
    deadline = time.time() + timeout
    while time.time() < deadline:
        if cli.wait_for_service(timeout_sec=1.0):
            f = cli.call_async(GetState.Request()); t = time.time()
            while not f.done() and time.time() - t < 2:
                time.sleep(0.05)
            if f.done() and f.result().current_state.label == 'active':
                return True
        time.sleep(0.5)
    return False


def near(pts, c, r=0.3):
    return any(math.hypot(px - c[0], py - c[1]) < r for px, py in pts)


class Suite:
    def __init__(self, nav, mon):
        self.nav, self.mon, self.results = nav, mon, []

    def ps(self, x, y, yaw=0.0):
        p = PoseStamped(); p.header.frame_id = 'map'; p.header.stamp = self.nav.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return p

    def run_task(self, timeout, on_tick=None):
        """Spin until the current task finishes. Returns (result, seconds, max_recoveries)."""
        t0 = time.time(); rec = 0
        while not self.nav.isTaskComplete():
            fb = self.nav.getFeedback()
            if fb is not None and hasattr(fb, 'number_of_recoveries'):
                rec = max(rec, fb.number_of_recoveries)
            if on_tick:
                on_tick(time.time() - t0)
            if time.time() - t0 > timeout:
                self.nav.cancelTask()
                return 'TIMEOUT', time.time() - t0, rec
        return self.nav.getResult().name, time.time() - t0, rec

    def record(self, name, expected, verdict, **m):
        xyz, rpy = gt_pose()
        loc = self.mon.pose()
        with self.mon.lock:
            m.update(max_cmd_vx=round(self.mon.max_vx, 3), max_cmd_wz=round(self.mon.max_wz, 3),
                     closest_scan_m=round(self.mon.min_scan, 3),
                     collision_monitor=sorted(self.mon.cm_triggers),
                     speed_limits_ok=self.mon.max_vx <= VX_MAX + EPS and self.mon.max_wz <= WZ_MAX + EPS)
        m['gt_xy'] = [round(xyz[0], 3), round(xyz[1], 3)]; m['gt_yaw'] = round(rpy[2], 3)
        m['gt_roll_pitch_max'] = round(max(abs(rpy[0]), abs(rpy[1])), 3)
        if loc:
            m['loc_err_m'] = round(math.hypot(loc[0] - xyz[0], loc[1] - xyz[1]), 3)
            m['loc_yaw_err'] = round(abs(wrap(loc[2] - rpy[2])), 3)
        if not m['speed_limits_ok'] or m['gt_roll_pitch_max'] > 0.2:
            verdict = 'FAIL'
        r = dict(test=name, expected=expected, verdict=verdict, **m)
        print(json.dumps(r), flush=True)
        self.results.append(r)
        self.mon.reset()

    def goal(self, name, x, y, yaw=0.0, timeout=150, expected='SUCCEEDED, final pose within tolerance', check_yaw=False):
        self.nav.goToPose(self.ps(x, y, yaw))
        res, dt, rec = self.run_task(timeout)
        xyz, rpy = gt_pose()
        err = math.hypot(xyz[0] - x, xyz[1] - y); yerr = abs(wrap(rpy[2] - yaw))
        ok = res == 'SUCCEEDED' and err <= XY_TOL + 0.05 and (not check_yaw or yerr <= YAW_TOL + 0.05)
        self.record(name, expected, 'PASS' if ok else 'FAIL', result=res, secs=round(dt, 1), recoveries=rec,
                    goal_xy_err=round(err, 3), goal_yaw_err=round(yerr, 3))
        return res

    # ---------------------------------------------------------------- tests
    def t_demo_goal1(self, name='demo_goal1_unmapped_box'):
        self.nav.goToPose(self.ps(1.8, 0.5))
        res, dt, rec = self.run_task(150)
        xyz, _ = gt_pose()
        with self.mon.lock:
            plans = list(self.mon.plans)
        before = [p for t, rp, p in plans if rp and math.hypot(rp[0] - BOX1[0], rp[1] - BOX1[1]) > 2.5]
        through_before = any(near(p, BOX1) for p in before)
        last_avoid = bool(plans) and not any(near(p, BOX1) for _, _, p in plans[-3:])
        switch_t = next((round(t - plans[0][0], 1) for t, _, p in plans if through_before and not near(p, BOX1)), None)
        err = math.hypot(xyz[0] - 1.8, xyz[1] - 0.5)
        exercised = through_before and last_avoid
        v = 'FAIL' if res != 'SUCCEEDED' or err > XY_TOL + 0.05 else ('PASS' if exercised else 'NOT EXERCISED')
        self.record(name, 'SUCCEEDED; plan first goes through box gap, then reroutes after lidar sees it', v,
                    result=res, secs=round(dt, 1), recoveries=rec, goal_xy_err=round(err, 3),
                    plans=len(plans), plan_through_box_before_seen=through_before,
                    final_plans_avoid_box=last_avoid, reroute_after_s=switch_t)

    def t_demo_goal2(self, name='demo_goal2_live_box', box_name='live_obstacle'):
        state = {'spawned': None, 'pre': []}

        def tick(el):
            p = self.mon.pose()
            if state['spawned'] is None and p and p[0] < 0.6:
                with self.mon.lock:
                    state['pre'] = [pl for _, _, pl in self.mon.plans[-3:]]
                state['spawned'] = (spawn(box_name, *LIVE), round(el, 1), round(p[0], 2), round(p[1], 2))
        self.nav.goToPose(self.ps(-2.0, -0.5))
        res, dt, rec = self.run_task(150, tick)
        xyz, _ = gt_pose()
        with self.mon.lock:
            plans = list(self.mon.plans)
        pre_through = any(near(p, LIVE) for p in state['pre'])
        last_avoid = bool(plans) and not any(near(p, LIVE) for _, _, p in plans[-3:])
        err = math.hypot(xyz[0] + 2.0, xyz[1] + 0.5)
        sp = state['spawned']
        if res != 'SUCCEEDED' or err > XY_TOL + 0.05:
            v = 'FAIL'
        else:
            v = 'PASS' if (sp and sp[0] and pre_through and last_avoid) else 'NOT EXERCISED'
        self.record(name, 'SUCCEEDED; box spawned ahead on planned route, robot reroutes', v,
                    result=res, secs=round(dt, 1), recoveries=rec, goal_xy_err=round(err, 3),
                    spawn_ok_at_s_x_y=sp, plan_through_spot_before_spawn=pre_through, final_plans_avoid_box=last_avoid)

    def t_goal_at_current_pose(self):
        p = self.mon.pose()
        self.goal('goal_at_current_pose', p[0], p[1], p[2], timeout=30,
                  expected='SUCCEEDED almost immediately (no motion needed)')

    def t_cancel(self):
        self.nav.goToPose(self.ps(1.7, 1.4))
        t0 = time.time()
        while time.time() - t0 < 6:
            self.nav.isTaskComplete()
        moving = abs(self.mon.odom_v[0]) > 0.05 or abs(self.mon.odom_v[1]) > 0.1
        self.nav.cancelTask(); tc = time.time(); stop_s = None
        while time.time() - tc < 5:
            if stop_s is None and abs(self.mon.odom_v[0]) < 0.01 and abs(self.mon.odom_v[1]) < 0.05:
                stop_s = round(time.time() - tc, 2)
            time.sleep(0.05)
        a, _ = gt_pose(); time.sleep(3); b, _ = gt_pose()
        drift = math.hypot(a[0] - b[0], a[1] - b[1])
        self.nav.isTaskComplete()
        ok = moving and stop_s is not None and stop_s <= 2.0 and drift < 0.02
        self.record('cancel_mid_route', 'robot stops within ~2 s of cancel and stays stopped',
                    'PASS' if ok else ('NOT EXERCISED' if not moving else 'FAIL'),
                    result=str(self.nav.getResult().name), was_moving=moving, stop_after_s=stop_s,
                    drift_3s_after_m=round(drift, 3))

    def t_preempt(self):
        self.nav.goToPose(self.ps(1.8, -1.0))
        t0 = time.time()
        while time.time() - t0 < 6:
            self.nav.isTaskComplete()
        self.goal('preempt_new_goal_mid_route', -1.6, 1.4, math.pi / 2,
                  expected='second goal replaces the first; robot ends at the second goal')

    def t_enclosed(self):
        names = ['encl_s', 'encl_n', 'encl_w']
        spots = [(0.55, 0.0), (0.55, 1.1), (0.0, 0.55)]
        # Drive to a staging pose first so no box is spawned on top of the robot.
        self.nav.goToPose(self.ps(-1.6, -1.4))
        self.run_task(150)
        xyz, _ = gt_pose()
        clear = min(math.hypot(xyz[0] - sx, xyz[1] - sy) for sx, sy in spots)
        self.mon.reset()
        ok = clear > 1.0 and all(spawn(n, *s) for n, s in zip(names, spots))
        time.sleep(1)
        self.nav.goToPose(self.ps(0.55, 0.55))
        res, dt, rec = self.run_task(420)
        xyz, _ = gt_pose()
        inside = abs(xyz[0] - 0.55) < 0.4 and abs(xyz[1] - 0.55) < 0.4
        v = ('PASS' if res == 'FAILED' and not inside else 'FAIL') if ok else 'NOT EXERCISED'
        self.record('goal_enclosed_by_boxes', 'ABORTED after recoveries, no collision, robot stays outside', v,
                    result=res, secs=round(dt, 1), recoveries=rec, spawned=ok, robot_clearance_at_spawn_m=round(clear, 2),
                    robot_inside_enclosure=inside)
        return names

    def t_cleared(self, names):
        removed = all(remove(n) for n in names)
        time.sleep(1)
        res = self.goal('goal_after_boxes_removed', 0.55, 0.55, timeout=150,
                        expected='SUCCEEDED: costmap clears removed boxes')
        if not removed:
            self.results[-1]['verdict'] = 'FAIL'; self.results[-1]['removed'] = False

    def t_unreachable(self, name, x, y, timeout, expected):
        self.nav.goToPose(self.ps(x, y))
        res, dt, rec = self.run_task(timeout)
        xyz, _ = gt_pose()
        self.record(name, expected, 'PASS' if res == 'FAILED' else 'INFO', result=res, secs=round(dt, 1),
                    recoveries=rec, final_dist_to_goal=round(math.hypot(xyz[0] - x, xyz[1] - y), 3))

    def t_through_poses(self):
        poses = [self.ps(-0.55, -1.6), self.ps(0.55, -0.55), self.ps(1.7, 1.4, math.pi / 2)]
        self.nav.goThroughPoses(poses)
        res, dt, rec = self.run_task(200)
        xyz, rpy = gt_pose()
        err = math.hypot(xyz[0] - 1.7, xyz[1] - 1.4)
        self.record('navigate_through_poses', 'SUCCEEDED, ends at last pose', 'PASS' if res == 'SUCCEEDED' and err <= 0.3 else 'FAIL',
                    result=res, secs=round(dt, 1), recoveries=rec, goal_xy_err=round(err, 3))

    def t_waypoints(self):
        pts = [(0.55, 1.6), (-0.55, 0.55), (-1.6, -1.4)]
        self.nav.followWaypoints([self.ps(*p) for p in pts])
        res, dt, _ = self.run_task(300)
        r = self.nav.result_future.result().result if self.nav.result_future and self.nav.result_future.result() else None
        missed = len(getattr(r, 'missed_waypoints', [])) if r else None
        xyz, _ = gt_pose()
        err = math.hypot(xyz[0] + 1.6, xyz[1] + 1.4)
        self.record('follow_waypoints', 'SUCCEEDED, no missed waypoints', 'PASS' if res == 'SUCCEEDED' and missed == 0 and err <= 0.3 else 'FAIL',
                    result=res, secs=round(dt, 1), missed_waypoints=missed, goal_xy_err=round(err, 3))

    def t_goal_pose_topic(self, x=-2.0, y=-0.5):
        """RViz '2D Goal Pose' path: publish on /goal_pose; no action handle, so watch ground truth."""
        while self.mon.count_subscribers('goal_pose') == 0:
            time.sleep(0.2)
        self.mon.goal_pub.publish(self.ps(x, y))
        t0 = time.time(); err = None
        while time.time() - t0 < 150:
            xyz, _ = gt_pose()
            err = math.hypot(xyz[0] - x, xyz[1] - y)
            v, w = self.mon.odom_v
            if err <= XY_TOL + 0.05 and abs(v) < 0.01 and abs(w) < 0.05:
                break
            time.sleep(1)
        dt = time.time() - t0
        self.record('goal_via_goal_pose_topic', 'robot reaches goal sent like RViz 2D Goal Pose and stops',
                    'PASS' if err <= XY_TOL + 0.05 and dt < 150 else 'FAIL', secs=round(dt, 1), goal_xy_err=round(err, 3))

    def t_startup(self, active_s):
        xyz, _ = gt_pose(); p = self.mon.pose()
        err = math.hypot(p[0] - xyz[0], p[1] - xyz[1]) if p else None
        self.record('startup_and_localization', 'Nav2 active, map->base_footprint TF present, AMCL at spawn pose',
                    'PASS' if active_s is not None and p and err < 0.1 else 'FAIL', nav2_active_after_s=active_s,
                    init_loc_err_m=round(err, 3) if err is not None else None)


def main():
    mode, out = sys.argv[1], sys.argv[2]
    rclpy.init()
    mon = Monitor()
    ex = MultiThreadedExecutor(); ex.add_node(mon)
    threading.Thread(target=ex.spin, daemon=True).start()
    nav = BasicNavigator()
    t0 = time.time()
    # Not nav.waitUntilNav2Active(): it never times out, and with localizer='amcl' it publishes its own
    # initial pose (the map origin), which would reset AMCL onto the centre pillar.
    up = wait_active(mon, 'bt_navigator', 120)
    while up and mon.pose() is None and time.time() - t0 < 120:
        time.sleep(0.2)
    up = up and mon.pose() is not None
    active_s = round(time.time() - t0, 1) if up else None
    time.sleep(2)
    s = Suite(nav, mon); mon.reset()
    sim0, wall0 = mon.sim_t, time.time()
    s.t_startup(active_s)
    if not up:
        s.results[-1]['error'] = 'bt_navigator not active (or no map->base_footprint TF) within 120 s'
        json.dump({'mode': mode, 'real_time_factor': None, 'results': s.results}, open(out, 'w'), indent=1)
        print('DONE startup failed', flush=True)
        os._exit(1)
    s.t_demo_goal1()
    s.t_demo_goal2()
    if mode == 'full':
        s.results[-1]['live_box_removed_after'] = remove('live_obstacle')
        s.t_goal_at_current_pose()
        s.goal('orientation_goal_yaw_+90deg', 0.55, -0.55, math.pi / 2, check_yaw=True,
               expected='SUCCEEDED, position and heading within 0.25 m / 0.25 rad')
        for i, (x, y, yaw) in enumerate([(0.55, 1.7, 0.0), (2.0, -0.5, -math.pi / 2), (0.0, -1.8, math.pi), (-1.7, 1.4, math.pi / 2)]):
            s.goal(f'tour_{i + 1}_({x},{y})', x, y, yaw)
        s.t_through_poses()
        s.t_waypoints()
        s.t_cancel()
        s.t_preempt()
        s.t_unreachable('goal_inside_pillar_(0,0)', 0.0, 0.0, 150, 'observe: planner tolerance 0.5 m may let it end near the pillar')
        s.t_unreachable('goal_outside_walls_(3.5,0)', 3.5, 0.0, 150, 'ABORTED: no path through the walls')
        s.t_unreachable('goal_outside_map_(15,15)', 15.0, 15.0, 60, 'ABORTED quickly: goal off the costmap')
        names = s.t_enclosed()
        s.t_cleared(names)
        s.t_goal_pose_topic()
    rtf = (mon.sim_t - sim0) / max(time.time() - wall0, 1e-6)
    json.dump({'mode': mode, 'real_time_factor': round(rtf, 2), 'results': s.results}, open(out, 'w'), indent=1)
    print('DONE rtf', round(rtf, 2), flush=True)
    nav.destroy_node(); mon.destroy_node()
    rclpy.try_shutdown()
    os._exit(0)


if __name__ == '__main__':
    main()
