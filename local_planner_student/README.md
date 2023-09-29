

```bash
ros2 run stage_ros stageros src/stage_ros/world/maze.world
```
```bash
ros2 run local_planner_student local_planner
```

```bash
ros2 service call /goalService local_planner_srvs/srv/LocalGoal "{
  'goal_pose2d': {
    'x': 1.0,
    'y': 1.0,
    'theta': 0.0
  }
}"
```
