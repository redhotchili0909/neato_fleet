# Documentation

## Running Simulator

### Step 1:
Replace the contents of the file `ros2_ws/src/neato_packages/neato_node2/neato_node2/simulator_adapter.py` with the contents of the file `ros2_ws/src/neato_fleet/simulator_adapter.py`

### Step 2:
REMEMBER: Build and Source ROS2 Enviroment

Run Gazebo with: 
`ros2 launch neato_fleet multi_neato_world.py`

## Adding TwistArray msg

In `ros2_ws/src/neato_packages/neato2_interfaces/msg` add the file `TwistArray.msg`:

```
geometry_msgs/Twist[] twist_array
```

## Important Commands

### Connecting to multiple neatos (IRL):

```
ros2 launch neato_node2 bringup_multi.py host:=192.168.16.111 robot_name:=robot1 udp_video_port:=5002 udp_sensor_port:=7777 gscam_config:='udpsrc port=5002 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert'
```

```
ros2 launch neato_node2 bringup_multi.py host:=192.168.16.126 robot_name:=robot2 udp_video_port:=5003 udp_sensor_port:=7778 gscam_config:='udpsrc port=5003 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert'
```