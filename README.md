# Neato Fleet Control — Multi-Robot Collision Avoidance

*Chang Jun Park, Akshat Jain, Dhvan Shah, Zaraius Bilimoria*

## Overview

A ROS2 system for coordinating multiple Neato robots using Reciprocal Velocity Obstacles (RVO2) collision avoidance. Robots navigate autonomously toward individual goals without collisions, tested in both Gazebo simulation and on real hardware.

**For a detailed project report, methodology, and results, visit our [project website](https://neato-fleet.netlify.app/).**

## Quick Start

### Prerequisites
- ROS2 (tested with Humble)
- Python 3.9+
- `rclpy`, `geometry_msgs`, `nav_msgs`, `visualization_msgs`
- RVO2 library (included in `external/Python-RVO2`)

### Installation
```bash
cd ros2_ws/src
git clone <this-repo>
cd ../..
colcon build
source install/setup.bash
```

### Simulation (Gazebo)

**Edit the config file** to set your robot positions, goals, and scenario:
```bash
vim config/fleet_config.yaml
```

**Launch everything with one command:**
```bash
ros2 launch neato_fleet rvo_fleet.py
```

This automatically launches:
- Gazebo with 3 simulated Neatos
- RVO fleet controller (reads odometry, computes safe velocities)
- Goal publisher (publishes target positions based on YAML)
- Command recorder (logs trajectories for playback)

Change the scenario by editing the `scenario` parameter in `config/fleet_config.yaml`:
```yaml
/fleet_goals:
  ros__parameters:
    scenario: "swap"  # swap, straight, cross, home, or pillar
```

### Real Hardware

The typical workflow is to **record trajectories in simulation**, then **play them back on real robots**.

**Step 1: Record in simulation**

Run the simulation and let `cmd_recorder.py` capture the trajectory (it runs automatically with `rvo_fleet.py`). The recorded CSV file will be saved in the workspace.

**Step 2: Connect to each Neato**

Open a separate terminal for each robot and launch with unique namespaces and ports:

**Robot 1:**
```bash
ros2 launch neato_node2 bringup_multi.py host:=ip1 robot_name:=robot1 \
  udp_video_port:=5002 udp_sensor_port:=7777 \
  gscam_config:='udpsrc port=5002 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert'
```

**Robot 2:**
```bash
ros2 launch neato_node2 bringup_multi.py host:=ip2 robot_name:=robot2 \
  udp_video_port:=5003 udp_sensor_port:=7778 \
  gscam_config:='udpsrc port=5003 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert'
```

**Robot 3:**
```bash
ros2 launch neato_node2 bringup_multi.py host:=ip3 robot_name:=robot3 \
  udp_video_port:=5004 udp_sensor_port:=7779 \
  gscam_config:='udpsrc port=5004 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert'
```

*Replace the IP addresses with your actual Neato IPs. Each robot needs a unique `robot_name`, `udp_video_port`, and `udp_sensor_port`.*

**Step 3: Play back the recorded trajectory**

```bash
ros2 run neato_fleet cmd_player --ros-args -p trajectory_file:=cmd_trajectory_YYYYMMDD_HHMMSS.csv
```

This replays the exact commands from simulation on your real robots, bypassing the need for odometry feedback.

## ROS2 Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/robot{i}/odom` | `nav_msgs/Odometry` | Robot position and velocity (subscribe) |
| `/robot{i}/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (publish) |
| `/fleet_goals` | `visualization_msgs/MarkerArray` | Goal visualization in RViz |

## Configuration

All parameters are defined in `config/fleet_config.yaml`. Edit this file to customize robot positions, goals, and RVO behavior.

### RVO Fleet Controller Parameters

Under `/rvo_fleet_controller`:
- `num_robots` — Number of robots in the fleet
- `control_rate` — Control loop frequency (Hz)
- `max_speed` — Maximum velocity (m/s)
- `robot_radius` — Physical robot radius (m)
- `time_horizon` — How far ahead RVO2 plans (seconds)
- `neighbor_dist` — How far to look for other robots (meters)
- `start_positions` — Initial [x1, y1, x2, y2, ...] coordinates

### Fleet Goals Parameters

Under `/fleet_goals`:
- `scenario` — Active scenario name (swap, straight, cross, home, pillar)
- `goals_<scenario>` — Goal positions for each scenario as [x1, y1, x2, y2, ...]

Example configuration:
```yaml
/rvo_fleet_controller:
  ros__parameters:
    num_robots: 3
    control_rate: 20.0
    max_speed: 0.2
    robot_radius: 0.35
    time_horizon: 2.0
    neighbor_dist: 10.0
    start_positions: [0.8, 2.8, 0.8, 2.0, 0.8, 1.2]

/fleet_goals:
  ros__parameters:
    scenario: "swap"
    goals_swap: [3.8, 2.0, 2.9, 2.0, 2.0, 2.0]
    goals_cross: [3.8, 3.0, 2.9, 4.0, 2.0, 3.0]
```

### Creating Custom Scenarios

You can add your own scenarios by adding new goal lists to `config/fleet_config.yaml`:

```yaml
/fleet_goals:
  ros__parameters:
    scenario: "my_custom"  # Set your scenario name
    goals_my_custom: [5.0, 5.0, 10.0, 10.0, 15.0, 5.0]  # Custom goal positions
```

The format is `[x1, y1, x2, y2, ...]` where each `(xi, yi)` is the goal position for robot `i`. Make sure the number of goals matches `num_robots`.

### Using More Robots

You can scale up to any number of robots by updating `num_robots` and providing corresponding start/goal positions:

```yaml
/rvo_fleet_controller:
  ros__parameters:
    num_robots: 5  # Increase this
    start_positions: [0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 3.0, 0.0, 4.0, 0.0]  # 5 robots

/fleet_goals:
  ros__parameters:
    scenario: "my_scenario"
    goals_my_scenario: [4.0, 0.0, 3.0, 0.0, 2.0, 0.0, 1.0, 0.0, 0.0, 0.0]  # 5 goals
```

The controller will automatically handle any number of robots, just ensure your lists have `2 * num_robots` values (x, y pairs).

## What Each Script Does

### `rvo_fleet_controller.py`
Main control loop that subscribes to `/robot{i}/odom`, computes collision-free velocities using RVO2, and publishes safe commands to `/robot{i}/cmd_vel`. The RVO2 algorithm handles collision math—this just bridges ROS2 with RVO2.

### `fleet_goals.py`
Reads goal positions from YAML config and publishes them as `MarkerArray` to `/fleet_goals` for RViz visualization. Change scenarios by editing the config file.

### `cmd_recorder.py`
Records all `/robot{i}/cmd_vel` commands to CSV during simulation runs. Useful for analyzing trajectories or replaying them on real hardware.

### `cmd_player.py`
Replays recorded CSV trajectories on real robots, bypassing odometry feedback (useful when odometry is noisy).

### `simulator_adapter.py`
Bridges Gazebo simulation and real hardware by standardizing odometry formats and velocity command handling.

## Setup Notes

### For Simulation
The `rvo_fleet.py` launch file already handles everything. Make sure your config file has valid start positions and goal positions for your chosen scenario.

### For Real Hardware
Ensure each Neato is running `neato_node2` bringup with a unique `robot_name` and publishing odometry to `/robot{i}/odom`. Update `config/fleet_config.yaml` with the actual physical positions where you've placed the robots so the controller initializes correctly.

### Simulator Adapter
The `simulator_adapter.py` provides consistent interfaces between Gazebo simulation and real hardware. It's automatically used when running in simulation.

## Troubleshooting

**Robots not moving:**
- Check that the controller is running: `ros2 node list`
- Verify odometry is being published: `ros2 topic echo /robot1/odom`
- Check that goals are published: `ros2 topic echo /fleet_goals`

**Robots ignoring goals:**
- Check that `scenario` in config matches an available goal set
- Verify goal positions are within the simulation/real-world bounds

**Collisions occurring:**
- Increase `time_horizon` in config (longer lookahead)
- Increase `neighbor_dist` (broader awareness of other robots)
- Reduce `max_speed` (slower = more time to avoid)

**Jerky motion:**
- Reduce `max_speed`
- Lower `control_rate` for smoother updates
- Check for high network latency or odometry noise

**Command Recording/Playback Issues:**
- Ensure trajectory file is in the working directory
- Check that the number of robots in config matches the recorded trajectory

