#!/usr/bin/env python3
"""Print Markdown tables from tests/out/*/ (or the dirs given). Usage: summarize.py [run_dir ...]"""
import glob, json, os, sys

dirs = sys.argv[1:] or sorted(glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out', '*')))
KEYS = ['result', 'secs', 'recoveries', 'goal_xy_err', 'goal_yaw_err', 'reroute_after_s', 'stop_after_s',
        'final_dist_to_goal', 'missed_waypoints', 'nav2_active_after_s', 'robot_inside_enclosure']

for d in dirs:
    try:
        res = json.load(open(os.path.join(d, 'results.json')))
        sd = json.load(open(os.path.join(d, 'shutdown.json')))
    except FileNotFoundError:
        continue
    print(f"### {os.path.basename(d)} ({res['mode']}, real-time factor {res['real_time_factor']})\n")
    print('| Test | Verdict | Details | Max cmd vx / wz | Closest scan (m) | Loc. err (m) |')
    print('|---|---|---|---|---|---|')
    for r in res['results']:
        det = ', '.join(f'{k}={r[k]}' for k in KEYS if k in r)
        print(f"| {r['test']} | {r['verdict']} | {det} | {r['max_cmd_vx']} / {r['max_cmd_wz']} "
              f"| {r['closest_scan_m']} | {r.get('loc_err_m')} |")
    print(f"\nShutdown (SIGINT): launch exit {sd['launch_exit']}, {sd['shutdown_ms']} ms, "
          f"{sd['leftover_procs']} leftover processes, crashed: {sd['crashed_on_shutdown'] or 'none'}\n")
