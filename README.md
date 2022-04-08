<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_sweep_scanner`
==================================

### Build Instruction
```bash
source /opt/ros/noetic/setup.bash
# Prepare catkin workspace (only need to be done once)
mkdir -p catkin_ws/src && cd catkin_ws
catkin_make
# Clone this repository
cd src && git clone https://github.com/107-systems/l3xz_sweep_scanner && cd ..
# Invoke catkin_make from the catkin workspace root.
catkin_make
```

### Scanse Sweep 2D Laser Scanner
```bash
source devel/setup.bash
roslaunch l3xz_sweep_scanner laser.launch
rviz --display-config src/l3xz/rviz/laser.rviz
```
**Note**: Possible you need to configure the right USB port in `launch`/[`laser.launch`](launch/laser.launch).
