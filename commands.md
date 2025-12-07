# Important Commands

## Connecting to Real Neatos

Each Neato needs its own terminal. Make sure to use unique `robot_name`, `udp_video_port`, and `udp_sensor_port` for each robot.

**Robot 1:**
```bash
ros2 launch neato_node2 bringup_multi.py host:=192.168.16.111 robot_name:=robot1 udp_video_port:=5002 udp_sensor_port:=7777 gscam_config:='udpsrc port=5002 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert'
```

**Robot 2:**
```bash
ros2 launch neato_node2 bringup_multi.py host:=192.168.16.126 robot_name:=robot2 udp_video_port:=5003 udp_sensor_port:=7778 gscam_config:='udpsrc port=5003 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert'
```

**Robot 3:** (update IP as needed)
```bash
ros2 launch neato_node2 bringup_multi.py host:=192.168.16.XXX robot_name:=robot3 udp_video_port:=5004 udp_sensor_port:=7779 gscam_config:='udpsrc port=5004 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert'
```

---

## Simulation (Gazebo)

Launch 3 simulated Neatos in Gazebo:
```bash
ros2 launch neato_fleet multi_neato_world.py
```

---

## RVO Fleet Controller


**Basic usage (default start positions):**
```bash
ros2 run neato_fleet rvo_fleet_controller
```

**With custom start positions** (for real hardware - specify where you physically placed the robots):
```bash
# Format: [x1,y1, x2,y2, x3,y3]
ros2 run neato_fleet rvo_fleet_controller --ros-args \
    -p num_robots:=3 \
    -p start_positions:="[0.0,0.0, 2.0,0.0, 4.0,0.0]"
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_robots` | 3 | Number of robots to control |
| `start_positions` | [0,0, 2,0, 4,0] | Starting positions [x1,y1, x2,y2, ...] |
| `max_speed` | 0.5 | Maximum speed (m/s) |
| `robot_radius` | 0.2 | Robot radius for collision avoidance (m) |
| `goal_tolerance` | 0.1 | How close to goal before stopping (m) |
| `neighbor_dist` | 3.0 | How far to look for other robots (m) |
| `time_horizon` | 2.0 | How far ahead to plan (seconds) |

---

## Fleet Goals

Publishes goal positions to make robots move in different patterns.

**Available scenarios:**
```bash
# Swap: Robots exchange positions (robot1 ↔ robot3)
ros2 run neato_fleet fleet_goals --ros-args -p scenario:=swap

# Cross: Robots cross through center point
ros2 run neato_fleet fleet_goals --ros-args -p scenario:=cross

# Home: All robots return to start positions
ros2 run neato_fleet fleet_goals --ros-args -p scenario:=home
```

**With custom number of robots:**
```bash
ros2 run neato_fleet fleet_goals --ros-args -p scenario:=swap -p num_robots:=2
```


---

## Quick Start (Simulation)

```bash
# Terminal 1: Launch Gazebo simulation
ros2 launch neato_fleet multi_neato_world.py

# Terminal 2: Start fleet controller
ros2 run neato_fleet rvo_fleet_controller

# Terminal 3: Send goals
ros2 run neato_fleet fleet_goals --ros-args -p scenario:=swap
```

```
ros2 run neato_fleet cmd_recorder --ros-args -p num_robots:=3
```

``` 
ros2 run neato_fleet cmd_player --ros-args   -p num_robots:=3   -p trajectory_file:=cmd_trajectory_20251207_164558.csv
```

## Quick Start (Real Hardware)

```bash
# Terminal 1-3: Connect to each Neato (see above)

# Terminal 4: Start fleet controller with your start positions
ros2 run neato_fleet rvo_fleet_controller --ros-args \
    -p num_robots:=3 \
    -p start_positions:="[0.3048,0.3048, 1.2192,0.3048, 2.1336,0.3048]"

# Terminal 5: Send goals
ros2 run neato_fleet fleet_goals --ros-args -p scenario:=swap -p num_robots:=3
```

```
ros2 run neato_fleet rvo_fleet_controller --ros-args     -p num_robots:=3     -p start_positions:="[0.0,0.0, 0.9144,0.0, 1.8288,0.0]"     -p start_yaws:="[1.5708, 1.5708, 1.5708]"
```
