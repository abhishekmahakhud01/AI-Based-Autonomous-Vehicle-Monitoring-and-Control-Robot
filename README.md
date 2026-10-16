# AI-Based Autonomous Vehicle Monitoring and Control Robot

ROS 2 Humble + Ubuntu 22.04 + Gazebo Classic simulation with a LiDAR reactive controller and a Flask telemetry dashboard.

**Default behavior:** the robot drives continuously and does not stop after 2 metres. When an obstacle is closer than 0.80 m in front, it rotates in place toward the side with more usable LiDAR clearance. It resumes forward movement after the front is clear beyond 0.95 m. If LiDAR data stops arriving or is invalid, it stops for safety.

This is reactive obstacle avoidance, not full SLAM/path planning. In a tight corner or a completely blocked area it may rotate in place; it does not guarantee finding a route through every maze.

## 1. Install dependencies once

```bash
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-turtlebot3-gazebo python3-flask gnome-terminal
```

## 2. Prepare and check

From the project directory:

```bash
chmod +x *.sh
./check_project.sh
```

Use Ubuntu 22.04 with ROS 2 Humble and Gazebo Classic. If `check_project.sh` reports a missing package or file, resolve that first.

## 3. Start the whole project

```bash
./start_project.sh
```

A Gazebo server, 3D client, robot spawn, dashboard, and controller terminal should open. Dashboard: <http://127.0.0.1:5000>.

For a slower VirtualBox VM, try:

```bash
./start_project.sh nogui
```

`nogui` skips the Gazebo 3D client; the simulation server and dashboard still run. This launcher still uses `gnome-terminal` to open its separate processes.

## 4. Continuous movement and optional travel limit

The default is unlimited travel (`ROBOT_MAX_DISTANCE=0`). Do not set a positive limit if you want continuous operation.

```bash
# Unlimited travel (default)
./run_controller.sh

# Optional: stop after 5 metres
ROBOT_MAX_DISTANCE=5 ./run_controller.sh
```

Only run **one controller at a time**. Two controllers publishing to `/cmd_vel` will conflict. Restarting the controller resets its travelled-distance counter.

## 5. Check ROS topics

After the robot has spawned, in a new terminal:

```bash
source /opt/ros/humble/setup.bash
./check_topics.sh
ros2 topic echo /scan --once
ros2 topic echo /odom --once
```

The controller subscribes to `/scan` and `/odom`, and publishes to `/cmd_vel`. If your setup uses different topic names, remap them or adjust the names in `obstacle_avoidance.py` and `dashboard.py` consistently.

## 6. Stop the project

```bash
./stop_project.sh
```

You can also press Ctrl+C in the controller terminal. The controller publishes a zero velocity command during shutdown.


## 7. Record data and generate six graphs

Run the robot simulation first, and wait until `/scan`, `/odom`, and `/cmd_vel` are active. In a **separate terminal** in the project folder, install Matplotlib once if needed:

```bash
sudo apt update
sudo apt install -y python3-matplotlib
```

Start the recorder:

```bash
chmod +x record_and_plot.sh
./record_and_plot.sh
```

Let the robot drive and avoid obstacles for as long as you want. When you want to finish recording, press **Ctrl+C in the recorder terminal** (not the controller terminal). It saves `robot_data.csv` and generates six PNG graphs in a timestamped folder under `data/`:

1. LiDAR obstacle distance vs time (front, left, right).
2. Linear velocity vs time (commanded and odometry).
3. Angular velocity vs time (commanded and odometry).
4. Robot X-Y trajectory from odometry.
5. X and Y position vs time.
6. Front obstacle clearance vs commanded linear speed.

These plots use recorded ROS data, not fabricated sample values. The recorder samples at 5 Hz and needs the ROS 2 environment sourced. The XY trajectory is odometry-based and may drift; the graphs are for simulation analysis, not certified safety evidence. Each run gets its own output folder.

## Troubleshooting

- **`/scan` is missing:** wait for the robot to spawn; check Gazebo plugin errors and `ros2 topic list`.
- **Robot does not move:** ensure the controller is running, `/scan` is updating, and no second controller is publishing to `/cmd_vel`.
- **Robot keeps rotating:** it may be in a corner or a narrow space. Check the left/right values in the dashboard and ensure the world has a passable route. This simple reactive controller is not a global planner.
- **Dashboard says waiting:** check that `/scan` and `/odom` exist and that Flask is installed in the same Python environment.
- **`Entity [burger] already exists`:** the robot is already spawned; do not spawn a second one.
- **Gazebo is slow/blank:** try `./start_project.sh nogui`, allocate more RAM to VirtualBox if possible, and enable 3D acceleration if supported.
- **`gnome-terminal` missing:** install it with `sudo apt install gnome-terminal`; the bundled multi-window launcher depends on it.

## Files

- `obstacle_avoidance.py` — continuous LiDAR-based reactive controller.
- `dashboard.py` — live sensor, odometry and velocity dashboard.
- `start_project.sh` / `stop_project.sh` — start and stop processes.
- `check_project.sh` / `check_topics.sh` — environment and topic checks.
- `worlds/obstacle_world.world` — obstacle course.
- `record_and_plot.py` / `record_and_plot.sh` — records ROS data and generates six graphs.

**Safety note:** this is a simulation/educational controller, not a certified real-robot safety system. Test at low speed and keep an independent emergency stop available for any physical robot.
