# Documentation

## Running Simulator

### Step 1:
Create a new file: `ros2_ws/src/neato_packages/neato2_gazebo/launch/multi_neato_world.py`

With contents:
```
#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg = get_package_share_directory('neato2_gazebo')
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg, 'worlds', 'test.world')

    sdf_path = os.path.join(
        pkg,
        'models',
        'neato',
        'neato_with_camera.sdf'
    )

    num_robots = 3   # change to however many you want

    spawn_nodes = []
    for i in range(num_robots):
        name = f"robot{i+1}"

        # Spawn Neato
        spawn_nodes.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', name,
                    '-robot_namespace', f'/{name}',
                    '-file', sdf_path,
                    '-x', str(i * 1.5),
                    '-y', '0',
                    '-z', '0.05',
                ],
                output='screen'
            )
        )

        # Simulator adapter (adds TF, stable_scan, accel, bump)
        spawn_nodes.append(
            Node(
                package='neato_node2',
                executable='simulator_adapter',
                namespace=name,
                parameters=[{'tf_prefix': name}],
                output='screen'
            )
        )

        # Optional: scan_to_pc2 fix node
        spawn_nodes.append(
            Node(
                package='fix_scan',
                executable='scan_to_pc2',
                namespace=name,
                output='screen'
            )
        )

    return LaunchDescription([

        # Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_path}.items()
        ),

        # Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gzclient.launch.py')
            )
        ),

        # Spawn robots + nodes
        *spawn_nodes,
    ])
```

### Step 2:
Change the file `ros2_ws/src/neato_packages/neato_node2/neato_node2/simulator_adapter.py` 

With these contents:
```
# simulator_adapter.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
from gazebo_msgs.msg import ContactsState
from neato2_interfaces.msg import Accel, Bump

import tf2_ros
from tf_transformations import euler_from_quaternion
import time

class SimulatorRelay(Node):
    def __init__(self):
        super().__init__('simulator_relay')

        # Parameter: optional tf_prefix (string). If provided, used to build frame names.
        self.declare_parameter('tf_prefix', '')
        tf_pref = self.get_parameter('tf_prefix').get_parameter_value().string_value or ''
        # ensure prefix does not end with slash
        self.tf_prefix = tf_pref.strip('/')
        if self.tf_prefix:
            self.get_logger().info(f'Using tf_prefix: {self.tf_prefix}')

        # Determine namespace the node is in (leading slash or empty)
        ns = self.get_namespace()  # returns '' or '/robot1'
        self.namespace = ns.strip('/')
        if self.namespace:
            self.get_logger().info(f'Node namespace: /{self.namespace}')

        # QoS for sensors
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Build fully-qualified topic names we want to subscribe to and publish from.
        # We subscribe to the scoped topics (namespace if present) so launching with PushRosNamespace is fine.
        def t(topic):
            # returns '/robotX/topic' if namespace exists, otherwise 'topic'
            return f'/{self.namespace}/{topic}' if self.namespace else topic

        self.scan_topic = t('scan')
        self.imu_topic = t('imu')
        self.bump_topic = t('bumper')  # matches your SDF remapping

        # Publishers (publish into same namespace)
        self.accel_pub = self.create_publisher(Accel, t('accel'), 10)
        self.scan_pub = self.create_publisher(LaserScan, t('stable_scan'), 10)
        self.bump_pub = self.create_publisher(Bump, t('bump'), 10)

        # Subscriptions
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_received, qos_profile)
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_received, qos_profile)
        self.bump_sub = self.create_subscription(ContactsState, self.bump_topic, self.contacts_received, qos_profile)

        # tf buffer + listener (useful if you need transforms)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"Subscribed topics: scan='{self.scan_topic}', imu='{self.imu_topic}', bumper='{self.bump_topic}'")

    def _frame_candidates(self, base_name):
        """
        Return a list of frame name candidates to try for lookups.
        e.g. base_name='base_laser_link' -> ['/robot1/base_laser_link', 'robot1/base_laser_link', 'base_laser_link', 'robot1::base_laser_link']
        Also include tf_prefix variations if provided.
        """
        candidates = []
        if self.namespace:
            candidates.append(f'/{self.namespace}/{base_name}')
            candidates.append(f'{self.namespace}/{base_name}')
        if self.tf_prefix:
            candidates.append(f'{self.tf_prefix}/{base_name}')
            candidates.append(f'/{self.tf_prefix}/{base_name}')
        candidates.append(base_name)
        # Gazebo model-scoped tf can show up like "model::link", include that too
        if self.namespace:
            candidates.append(f'{self.namespace}::{base_name}')
        return candidates

    def try_lookup_transform(self, target_frame, source_frame, time=rclpy.time.Time()):
        """
        Try multiple candidate names for target/source frames and return first transform found.
        Returns transform or raises exception from tf2 on failure.
        """
        target_candidates = self._frame_candidates(target_frame)
        source_candidates = self._frame_candidates(source_frame)

        last_exc = None
        # iterate combinations
        for t in target_candidates:
            for s in source_candidates:
                try:
                    # tf2 uses builtin time types; use 0 for latest
                    trans = self.tf_buffer.lookup_transform(
                        t, s, rclpy.time.Time())
                    return trans
                except Exception as e:
                    last_exc = e
                    # continue trying
        # If nothing found, re-raise the last exception
        raise last_exc

    # Callbacks: pass-through style
    def scan_received(self, msg: LaserScan):
        # simply republish (you can modify filtering here)
        try:
            self.scan_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish stable_scan: {e}')

    def imu_received(self, msg: Imu):
        try:
            self.accel_pub.publish(
                Accel(
                    accel_x=msg.linear_acceleration.x / 9.8,
                    accel_y=msg.linear_acceleration.y / 9.8,
                    accel_z=msg.linear_acceleration.z / 9.8
                )
            )
        except Exception as e:
            self.get_logger().warn(f'Failed to publish accel: {e}')

    def contacts_received(self, msg: ContactsState):
        try:
            if len(msg.states):
                self.bump_pub.publish(Bump(left_front=1, left_side=1, right_front=1, right_side=1))
            else:
                self.bump_pub.publish(Bump(left_front=0, left_side=0, right_front=0, right_side=0))
        except Exception as e:
            self.get_logger().warn(f'Failed to publish bump: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SimulatorRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Important Commands

### Connecting to multiple neatos (IRL):

```
ros2 launch neato_node2 bringup_multi.py host:=192.168.16.111 robot_name:=robot1 udp_video_port:=5002 udp_sensor_port:=7777 gscam_config:='udpsrc port=5002 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert'
```

```
ros2 launch neato_node2 bringup_multi.py host:=192.168.16.126 robot_name:=robot2 udp_video_port:=5003 udp_sensor_port:=7778 gscam_config:='udpsrc port=5003 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264  ! videoconvert'
```