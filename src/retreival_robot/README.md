# Retrieval robot: known-map cylinder navigation starter

A teaching/prototype stack for the existing ROS 2 Jazzy package **`retreival_robot`**.

## What runs

- `map_publisher.py`: publishes generated `/map` as `nav_msgs/OccupancyGrid`.
- `landmark_localizer.py`: planar EKF predicts pose from `/odom`, associates **XY positions** in `/viz/detections` with map landmarks, then broadcasts **`map -> odom`** and publishes `/localization/pose` (`PoseWithCovarianceStamped`).
- `motion_planner.py`: on a `/get_task` (`geometry_msgs/PoseStamped` in `map`) runs inflated-grid A*, prunes straight-line sections, publishes `/planned_path` (`nav_msgs/Path`). It clears old path immediately on task arrival.
- `navigator.py`: waits without moving for a valid `/planned_path`, then performs turn-go-turn wheel control on `/wheel_velocity_controller/commands`. Stops on a new task, empty/invalid path, TF failure, or arrival.
- Existing `simple_odometry.py` remains the **only** `odom -> base_link` publisher, and `robot_state_publisher` stays the only model TF publisher.

## Important limitations

1. **RGB is stored in landmark JSON but NOT observed by the provided cylinder detector.** Marker colors are visualization styling, not a color classifier. Localization therefore uses geometry-only data association. It can confuse similar patterns and is **not a globally initialized localization system**.
2. Initial robot pose in map must be approximately known; defaults to `(0,0,0)`. Gazebo launch spawn arguments are `(0,0,0.5)`; height does not matter to 2D pose. Keep initial map/odom aligned.
3. Localization is a **prototype geometric correction** (rigid least-squares + gated nearest neighbor + smoothing), **not an EKF or AMCL**. Requires >=3 visible cylinders and sensible landmark correspondence. Without observations, `map -> odom` stays at its previous correction and odometry still drives motion.
4. Planner has **no live obstacle avoidance, collision sensing, replanning during movement, or ball-target behavior**. Static obstacles only, a conservative 0.48m inflation radius which must be validated against robot footprint. Supervise motion. The robot's wide chassis may require larger clearance.
5. Path-following uses a basic waypoint controller and assumes positive wheel angular velocity moves both wheels forward and right wheel faster turns counterclockwise. Verify against Gazebo. The node is **not** Nav2.
6. The provided world is a **replacement sample arena** called `retrieval_arena`. The exact existing `retrieval_arena.sdf` was not provided, so **do not overwrite your original** without backup.
7. Because ROS topics have no transaction/ack handshake, path/task arrival can race in this starter. A production design should use a ROS action to track task IDs, preemption, and results.
8. Do **not** simultaneously run your existing `cylinder_perception.launch.py` with `navigation_stack.launch.py`: both start camera bridge, static TF and cylinder detector. This new stack launches them itself. The old perception launch also spawns the original three test cylinders; those are not in the reference map.

## Install

1. Extract into a separate directory; copy the provided `navigation/*.py` to `~/retrieval_ws/src/retreival_robot/navigation/`, `maps/*` to `.../maps/`, `worlds/retrieval_landmarks.sdf` to `.../worlds/`, and `launch/navigation_stack.launch.py` to `.../launch/`.
2. Add to `CMakeLists.txt` before `ament_package()` (preserve existing install blocks):

```cmake
install(PROGRAMS
  navigation/map_publisher.py
  navigation/landmark_localizer.py
  navigation/motion_planner.py
  navigation/navigator.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})
```

Your existing install(DIRECTORY launch worlds ...) should already include the new launch and world files.

3. Ensure executability: `chmod +x ~/retrieval_ws/src/retreival_robot/navigation/*.py`.
4. Change **one line** in your current `gazebo.retreival.launch.py`, without altering any other robot launch logic:

```python
world_file = os.path.join(package_share, 'worlds', 'retrieval_landmarks.sdf')
```

5. Build:

```bash
cd ~/retrieval_ws
colcon build --symlink-install --packages-select retreival_robot
source install/setup.bash
```

## Run (after stopping old simulation, RViz, joint_state_publisher_gui, and perception launch)

Terminal 1:
```bash
ros2 launch retreival_robot gazebo.retreival.launch.py
```

Terminal 2:
```bash
source ~/retrieval_ws/install/setup.bash
ros2 launch retreival_robot navigation_stack.launch.py
```

Terminal 3, test transforms:
```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /localization/pose --once
ros2 topic echo /map --once --field info
```

In RViz (with `use_sim_time=true`), Fixed Frame `map`, display Map `/map`, Path `/planned_path`, MarkerArray `/viz/detections`, PointCloud2 `/camera/points`, TF.

## Give the planner a task

Example command (point must be clear of cylinders and wall):

```bash
ros2 topic pub --once /get_task geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 2.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"
```

Monitor `ros2 topic echo /planned_path --once`. The robot stays stopped until it receives a nonempty path, turns, drives to each waypoint, and stops at the goal. Send a fresh task to interrupt a previous one.

## Regenerate map and world

The source `scripts/generate_arena.py` writes **world SDF, JSON landmark list, PGM and YAML occupancy map together** from one landmark table. They stay aligned in world coordinates. Run from extracted folder:

```bash
python3 scripts/generate_arena.py
```

Do not add cylinder models by hand without regenerating map and landmarks, or navigation will use stale geometry. This is NOT a general-purpose SDF-to-map converter for arbitrary models, mesh geometry, slopes, or moving obstacles.

## Topics / frames

`/joint_states` -> existing odometry `/odom`, `odom->base_link`;
`/camera/points` -> existing detector `/viz/detections` (base_link);
landmark localizer: `map->odom`, `/localization/pose`;
static map publisher: `/map`;
`/get_task` -> planner `/planned_path` -> navigator `/wheel_velocity_controller/commands`.

## Required packages

ROS 2 Jazzy with `rclpy`, `tf2_ros`, `ros_gz_bridge`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`, `std_msgs`, plus Python `numpy`, `scipy`, and `yaml` (PyYAML). Example:

```bash
sudo apt install python3-numpy python3-scipy python3-yaml ros-jazzy-tf2-ros ros-jazzy-ros-gz-bridge
```

## Notes on measurements and filters

- The detector's marker ID is **not** the map landmark ID; matching uses predicted XY geometry. Marker display color is **not** measured RGB. The JSON contains actual *SDF display colors*, but no automatic RGB detection occurs.
- The odometry message feeds the EKF *incremental* motion (prediction). Landmark center observations feed EKF measurement updates; matching uses Euclidean and Mahalanobis gating. This is an educational filter with approximate process/measurement noise, not an industrial localization implementation.
- Localization state is planar (x, y, yaw). Landmarks are projected into the base_link XY plane; camera TF correction has already happened inside the detector. A delayed marker is not replayed through a filter history: delayed observations older than the configured threshold are discarded, and an accepted observation is applied to the current estimate.
- Check your URDF wheel geometry: nominal wheel radius 0.1132 m, separation 0.76 m. Confirm that positive wheel commands drive forward on *both* sides, and that the footprint fits a 0.48 m map inflation clearance.
- Optional RGB integration later should use actual RGB samples (e.g. registered image/colorized point cloud), not Marker.color, and combine landmark color with geometric gating.
- No real-time robot safety guarantee is provided: Gazebo-only experimental control, static map, no live obstacle updates.
