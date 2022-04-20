<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_sweep_scanner`
==================================
ROS driver for Scanse Sweep 360° 2D LIDAR.

### How-to-build
```bash
# Clone this repository into catkin_ws/src.
git clone https://github.com/107-systems/l3xz_sweep_scanner
# Invoke catkin_make from the catkin workspace root.
source /opt/ros/noetic/setup.bash
catkin_make
```

### How-to-run
```bash
source devel/setup.bash
roslaunch l3xz_sweep_scanner laser.launch
rviz --display-config src/l3xz/rviz/laser.rviz
```
**Note**: Possible you need to configure the right USB port in `launch`/[`laser.launch`](launch/laser.launch).

### Interface Documentation
#### Published Topics
| Default Name | Type |
|:-:|:-:|
| `/l3xz/laser_frame` | [`sensor_msgs/LaserScan`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html) |

#### Parameters
| Name | Default | Description |
|:-:|:-:|-|
| `serial_port` | `/dev/ttyUSB0 ` | Serial port under which the Scanse Sweep is connected to the system. |
| `rotation_speed` | 1-10 | Rotations per second of the sensor head. |
| `sample_rate ` | 500, 750, 1000 | Laser scan rate in Hz, i.e. 500 = 500 laser distance measurements per second. |
| `frame_id` | `laser_frame` | Topic under which the is published. |
