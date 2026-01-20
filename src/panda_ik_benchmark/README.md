# panda_ik_benchmark

A small MoveIt2 (MoveItPy) benchmark:
1) sample N reachable Cartesian points p1..pN for Panda (N up to 30)
2) sample M IK solutions for each point (M up to 3000)
3) compare greedy (local) vs DP prefix-optimal (global / enumeration-equivalent)
4) write a unified `summary.json` containing t1..tN, s1..sN and optimization rates

## Build

```bash
cd ~/ws_moveit2
colcon build --packages-select panda_ik_benchmark
source install/setup.bash
```

## Run (recommended)

```bash
ros2 launch panda_ik_benchmark ik_benchmark.launch.py num_points:=3 seed:=7
```

Data will be saved under `./data/<timestamp>/`.

Key outputs
-----------
- `targets.json`: p0 + target Cartesian points
- `p1.json`..`pN.json`: IK solutions for each target point
- `summary.json`: unified report (greedy vs global prefix DP, time records, chosen IK, rates)
