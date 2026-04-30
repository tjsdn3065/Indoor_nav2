# indoor_nav2

Indoor navigation stack on ROS 2 Jazzy + Gazebo Harmonic + Nav2,
supporting TurtleBot3 Burger and Unitree Go2 in simulation.

> ⚠️ **Project status — archived.** Released as-is for reference. Go2
> navigation (AMCL, Voronoi, and RTAB-Map paths) is unreliable: the
> CHAMP balance controller shakes the LiDAR/camera enough to disturb
> localization and the planner. TurtleBot3 path is stable.

## Demo

<video src="https://github.com/user-attachments/assets/84b58da5-e574-474a-8b16-cac23a30ee57" controls></video>

## Composition

| Package | Role |
|---|---|
| `sim_bringup` | Gazebo world + robot spawn + `ros_gz_bridge` + RViz |
| `indoor_nav2_bringup` | Nav2 maps, params, launch variants, Voronoi roadmap builder |
| `indoor_nav2_rtabmap` | RTAB-Map mapping + localization |
| `nav2_voronoi_planner` | Custom Nav2 `GlobalPlanner` plugin (Voronoi roadmap + Dijkstra) |


## Environment

| Component | Version |
|---|---|
| OS        | Ubuntu 24.04 |
| ROS 2     | Jazzy Jalisco |
| Simulator | Gazebo Harmonic |

## Dependencies

```bash
sudo apt install \
    ros-jazzy-ros-gz \
    ros-jazzy-navigation2 \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-realsense2-description \
    ros-jazzy-velodyne-description \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-localization \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    python3-numpy python3-scipy python3-opencv python3-yaml
```

## Installation

```bash
# 1) Create the workspace and clone this repo into src/
mkdir -p ~/your_ws/src
cd    ~/your_ws/src
git clone https://github.com/tjsdn3065/indoor_nav2.git

# 2) Clone the external Go2 description + CHAMP controller
git clone https://github.com/khaledgabr77/unitree_go2_ros2.git

# 3) Resolve any remaining transitive dependencies
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
cd ~/your_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

Open each numbered command in its own terminal (each one needs
`source install/setup.bash` first).

### TurtleBot3 + AMCL (default)

```bash
# T1
ros2 launch sim_bringup sim.launch.py

# T2
ros2 launch indoor_nav2_bringup bringup.launch.py
```

### TurtleBot3 + NavFn (planner baseline)

```bash
ros2 launch sim_bringup sim.launch.py
ros2 launch indoor_nav2_bringup bringup_navfn.launch.py
```

### Unitree Go2 + AMCL (static map)

```bash
ros2 launch sim_bringup go2.launch.py
ros2 launch indoor_nav2_bringup bringup_go2_amcl.launch.py
```

### Unitree Go2 + Voronoi global planner

Uses the prebuilt roadmap `maps/small_house_roadmap_go2.json`.

```bash
ros2 launch sim_bringup go2.launch.py
ros2 launch indoor_nav2_bringup bringup_go2_voronoi.launch.py
```

### Unitree Go2 + RTAB-Map

Mapping (run once, then move the DB into the package and rebuild so
the localization launch can find it):

```bash
ros2 launch sim_bringup go2.launch.py
ros2 launch indoor_nav2_rtabmap rtabmap_mapping.launch.py
ros2 run  teleop_twist_keyboard teleop_twist_keyboard

mv ~/.ros/rtabmap.db ~/your_ws/src/indoor_nav2_rtabmap/databases/small_house_go2.db
colcon build --packages-select indoor_nav2_rtabmap
```

Navigation:

```bash
ros2 launch sim_bringup go2.launch.py
ros2 launch indoor_nav2_bringup bringup_go2.launch.py
```

## Building a Voronoi roadmap

The `nav2_voronoi_planner` plugin reads a precomputed roadmap JSON.
Roadmaps for the bundled `small_house` map are already in
`indoor_nav2_bringup/maps/`. To regenerate, or to build one for a
different map / robot radius:

```bash
cd ~/your_ws/src/indoor_nav2_bringup
python3 scripts/build_voronoi_roadmap.py \
    --map         maps/small_house.yaml \
    --output      maps/small_house_roadmap.json \
    --robot-radius 0.12 \
    --resampling-dist 0.4 \
    --debug        # optional: free_map.png + voronoi_*.png in *_debug/
```

## Acknowledgments

- [AWS RoboMaker small house](https://github.com/aws-robotics/aws-robomaker-small-house-world)
- [ROBOTIS TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [khaledgabr77/unitree_go2_ros2](https://github.com/khaledgabr77/unitree_go2_ros2)
  (CHAMP-based Go2 description + controllers)
