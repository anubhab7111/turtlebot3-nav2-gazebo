### full (full, real-time factor 0.99)

| Test | Verdict | Details | Max cmd vx / wz | Closest scan (m) | Loc. err (m) |
|---|---|---|---|---|---|
| startup_and_localization | PASS | nav2_active_after_s=7.3 | 0.0 / 0.0 | 0.484 | 0.001 |
| demo_goal1_unmapped_box | PASS | result=SUCCEEDED, secs=31.5, recoveries=0, goal_xy_err=0.151, reroute_after_s=9.2 | 0.26 / 1.717 | 0.346 | 0.051 |
| demo_goal2_live_box | PASS | result=SUCCEEDED, secs=32.9, recoveries=0, goal_xy_err=0.079 | 0.26 / 1.8 | 0.329 | 0.109 |
| goal_at_current_pose | PASS | result=SUCCEEDED, secs=0.0, recoveries=0, goal_xy_err=0.109, goal_yaw_err=0.034 | 0.0 / 0.0 | 0.523 | 0.123 |
| orientation_goal_yaw_+90deg | PASS | result=SUCCEEDED, secs=11.4, recoveries=0, goal_xy_err=0.131, goal_yaw_err=0.145 | 0.26 / 0.592 | 0.376 | 0.07 |
| tour_1_(0.55,1.7) | PASS | result=SUCCEEDED, secs=10.4, recoveries=0, goal_xy_err=0.074, goal_yaw_err=0.224 | 0.26 / 0.568 | 0.291 | 0.22 |
| tour_2_(2.0,-0.5) | PASS | result=SUCCEEDED, secs=11.9, recoveries=0, goal_xy_err=0.138, goal_yaw_err=0.05 | 0.26 / 0.348 | 0.333 | 0.072 |
| tour_3_(0.0,-1.8) | PASS | result=SUCCEEDED, secs=10.5, recoveries=0, goal_xy_err=0.154, goal_yaw_err=0.038 | 0.26 / 0.362 | 0.349 | 0.078 |
| tour_4_(-1.7,1.4) | PASS | result=SUCCEEDED, secs=18.0, recoveries=0, goal_xy_err=0.151, goal_yaw_err=0.109 | 0.26 / 0.411 | 0.342 | 0.071 |
| navigate_through_poses | PASS | result=SUCCEEDED, secs=43.5, recoveries=0, goal_xy_err=0.048 | 0.26 / 1.8 | 0.343 | 0.114 |
| follow_waypoints | PASS | result=SUCCEEDED, secs=54.5, goal_xy_err=0.082, missed_waypoints=0 | 0.26 / 1.8 | 0.287 | 0.038 |
| cancel_mid_route | PASS | result=CANCELED, stop_after_s=0.38 | 0.26 / 0.251 | 0.346 | 0.074 |
| preempt_new_goal_mid_route | PASS | result=SUCCEEDED, secs=26.1, recoveries=0, goal_xy_err=0.197, goal_yaw_err=0.103 | 0.26 / 1.8 | 0.368 | 0.049 |
| goal_inside_pillar_(0,0) | INFO | result=SUCCEEDED, secs=20.6, recoveries=1, final_dist_to_goal=0.552 | 0.26 / 1.8 | 0.344 | 0.032 |
| goal_outside_walls_(3.5,0) | PASS | result=FAILED, secs=22.0, recoveries=16, final_dist_to_goal=3.86 | 0.0 / 1.0 | 0.31 | 0.067 |
| goal_outside_map_(15,15) | PASS | result=FAILED, secs=0.0, recoveries=0, final_dist_to_goal=21.152 | 0.0 / 0.0 | 0.375 | 0.067 |
| goal_enclosed_by_boxes | PASS | result=FAILED, secs=295.8, recoveries=25, robot_inside_enclosure=False | 0.26 / 1.8 | 0.269 | 0.014 |
| goal_after_boxes_removed | PASS | result=SUCCEEDED, secs=11.1, recoveries=0, goal_xy_err=0.087, goal_yaw_err=0.183 | 0.26 / 0.536 | 0.308 | 0.067 |
| goal_via_goal_pose_topic | PASS | secs=23.0, goal_xy_err=0.093 | 0.26 / 1.8 | 0.247 | 0.06 |

Shutdown (SIGINT): launch exit 0, 1643 ms, 0 leftover processes, crashed: collision_monitor (-11)

### cold1 (demo, real-time factor 0.99)

| Test | Verdict | Details | Max cmd vx / wz | Closest scan (m) | Loc. err (m) |
|---|---|---|---|---|---|
| startup_and_localization | PASS | nav2_active_after_s=5.9 | 0.0 / 0.0 | 0.48 | 0.001 |
| demo_goal1_unmapped_box | PASS | result=SUCCEEDED, secs=33.1, recoveries=0, goal_xy_err=0.166, reroute_after_s=10.2 | 0.26 / 1.709 | 0.347 | 0.16 |
| demo_goal2_live_box | PASS | result=SUCCEEDED, secs=32.9, recoveries=0, goal_xy_err=0.037 | 0.26 / 1.8 | 0.313 | 0.06 |

Shutdown (SIGINT): launch exit 0, 1634 ms, 0 leftover processes, crashed: none

### cold2 (demo, real-time factor 0.99)

| Test | Verdict | Details | Max cmd vx / wz | Closest scan (m) | Loc. err (m) |
|---|---|---|---|---|---|
| startup_and_localization | PASS | nav2_active_after_s=6.0 | 0.0 / 0.0 | 0.478 | 0.001 |
| demo_goal1_unmapped_box | PASS | result=SUCCEEDED, secs=33.3, recoveries=0, goal_xy_err=0.143, reroute_after_s=10.3 | 0.26 / 1.722 | 0.36 | 0.041 |
| demo_goal2_live_box | PASS | result=SUCCEEDED, secs=34.4, recoveries=0, goal_xy_err=0.073 | 0.26 / 1.8 | 0.314 | 0.055 |

Shutdown (SIGINT): launch exit 0, 1610 ms, 0 leftover processes, crashed: none

### cold3 (demo, real-time factor 0.99)

| Test | Verdict | Details | Max cmd vx / wz | Closest scan (m) | Loc. err (m) |
|---|---|---|---|---|---|
| startup_and_localization | PASS | nav2_active_after_s=5.6 | 0.0 / 0.0 | 0.475 | 0.001 |
| demo_goal1_unmapped_box | PASS | result=SUCCEEDED, secs=31.9, recoveries=0, goal_xy_err=0.15, reroute_after_s=9.2 | 0.26 / 1.669 | 0.339 | 0.103 |
| demo_goal2_live_box | PASS | result=SUCCEEDED, secs=33.4, recoveries=0, goal_xy_err=0.087 | 0.26 / 1.8 | 0.332 | 0.024 |

Shutdown (SIGINT): launch exit 0, 3252 ms, 0 leftover processes, crashed: none

### cold4 (demo, real-time factor 0.99)

| Test | Verdict | Details | Max cmd vx / wz | Closest scan (m) | Loc. err (m) |
|---|---|---|---|---|---|
| startup_and_localization | PASS | nav2_active_after_s=5.9 | 0.0 / 0.0 | 0.488 | 0.001 |
| demo_goal1_unmapped_box | PASS | result=SUCCEEDED, secs=32.3, recoveries=0, goal_xy_err=0.167, reroute_after_s=10.3 | 0.26 / 1.712 | 0.354 | 0.155 |
| demo_goal2_live_box | PASS | result=SUCCEEDED, secs=32.6, recoveries=0, goal_xy_err=0.062 | 0.26 / 1.8 | 0.324 | 0.075 |

Shutdown (SIGINT): launch exit 0, 1633 ms, 0 leftover processes, crashed: collision_monitor (-11)

### cold5 (demo, real-time factor 0.99)

| Test | Verdict | Details | Max cmd vx / wz | Closest scan (m) | Loc. err (m) |
|---|---|---|---|---|---|
| startup_and_localization | PASS | nav2_active_after_s=5.6 | 0.0 / 0.0 | 0.474 | 0.001 |
| demo_goal1_unmapped_box | PASS | result=SUCCEEDED, secs=31.4, recoveries=0, goal_xy_err=0.153, reroute_after_s=10.3 | 0.26 / 1.675 | 0.332 | 0.067 |
| demo_goal2_live_box | PASS | result=SUCCEEDED, secs=32.4, recoveries=0, goal_xy_err=0.116 | 0.26 / 1.8 | 0.328 | 0.055 |

Shutdown (SIGINT): launch exit 0, 1631 ms, 0 leftover processes, crashed: none

