# ARC_RepairPlatform

## Overview

**ARC_RepairPlatform** is a collection of ROS 2 packages developed at the **Applied Research Center (ARC), Florida International University (FIU)** to drive a **UR‑16e robotic arm** equipped with a pneumatic tool‑changer for **remote, semi‑autonomous repair, cleaning, and inspection** in hazardous DOE nuclear facilities. The platform aims to **minimise human exposure to radiation and contaminated waste** by allowing operators to control every aspect of the system from a safe, remote location.

## Repository Layout

|Path|Description|
|---|---|
|`repair_interface/`|Custom ROS 2 interfaces: **RepairAction** (goal/feedback/result) and **EigenMsg** (Eigen `Vector3d` helper).|
|`ur16_repair/`|Nodes, launch files, and utilities that orchestrate the UR‑16e arm, tool changer, and perception pipeline.|
|`docs/` _(TBD)_|Design diagrams, demo media, and additional notes (to be populated).|

> **Heads‑up 📌** A Docker image is under active development for one‑command deployment

---

## Prerequisites

- **ROS 2 Humble** or newer (tested on Ubuntu 22.04).
    
- **UR ROS 2 driver** (`ur_robot_driver` ≥ Humble).  
    Tested with `ur16e` polyscope 5.13+.
    
- **rmw_zenoh** DDS implementation (for low‑latency Wi‑Fi transport).
    
- **Eigen 3**, **PCL**, **Open3D**, plus any extras listed in each package’s `package.xml` / `CMakeLists.txt` (run `rosdep`—see below).
    
- Laptop/host connected to the robot’s Wi‑Fi network _(static IP recommended)._  
    Example SSID: UR16_ARC.
    

---

## Quick Start

### 1 – Create & clone the workspace

```bash
mkdir -p ~/repair_platform_ws/src
cd ~/repair_platform_ws/src
git clone -b main https://github.com/scuar003/ARC_RepairPlatform.git ARC_RepairPlatform
```

### 2 – Install dependencies

```bash
cd ~/repair_platform_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3 – Build & source

```bash
colcon build --symlink-install
source install/setup.bash    # add to ~/.bashrc if desired
```

### 4 – Bring up the UR‑16e driver

```bash
#### --- RUN ZENOH ROUTER --- ####
zenoh
cd zenoh_ws/
rsc; srcws
ros2 run rmw_zeno <TAB TAB>
# Terminal 2 – robot control
source /opt/ros/humble/setup.bash
# If the UR driver has its own workspace:
source ~/ur_driver_ws/install/setup.bash

#SANTI PC Handyman_ws
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur16e \
    robot_ip:=<ROBOT_IP> \
    launch_rviz:=true
```

_Wait for “Robot ready to receive URScript” before continuing._

### 5 – Launch the repair platform

```bash
# Terminal 3 – ARC_RepairPlatform
source ~/repair_platform_ws/install/setup.bash
ros2 launch ur16_repair ur16_repair.launch.py
```

### 6 - Start Nuc Coms

```bash
#ssh into nuc 
ssh arc@<NUC_IP>
#in the ssh session 
tmux
```
#### terminal 1
```bash
cd ~/zenoh_ws
#source 
rsc; srcws
ros2 run rmw_zenoh_cpp rmw_zenohd
```

#### terminal 2
```bash
cd ~/ur16_ws
rsc; srcws
ros2 launch robot_control r_contron.launch.py
```
#### terminal 3
```bash
cd ~/realsense_lidar
#source
rsc; srcws
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true
```
#### terminal 4
```bash
cd ~/rplidar
rsc; srcws
ros2 launch rplidar_ros rplidar_s2_launch.py
```

This launch file starts:

- **Tool server** & action server (handles `RepairAction` goals).
    
- Perception / point‑cloud fusion nodes.
    
- Interactive‑marker UI for task selection.
    

### 6 – Visualise in RViz 2

1. Open RViz (if not auto‑launched).
    
2. Set **Fixed Frame** ➜ `base_link`.
    
3. Add displays: **TF**, **Marker**, **Interactive Marker**, **PointCloud2**.
    
4. Confirm the live tool‑changer TF tree & point clouds align.
    

### 7 – Power & pneumatics checklist

- **Main power** and **motor power** must be **ON** even if the rover wheels are stationary—the pneumatic tool‑changer draws its supply from the motor bus.
    
- Verify airline pressure ≥ 90 psi and tool magazine is latched.
    

---

## Package Details

### `repair_interface`

|Artifact|Purpose|
|---|---|
|`RepairAction`|Sends high‑level commands to the action server (e.g. `grind`, `wipe`, `inspect`) along with a `geometry_msgs/PoseArray` describing the work‑area corners.|
|`EigenMsg`|Lightweight wrapper around an Eigen `Vector3d` for topics where an explicit 3‑vector is more convenient than `geometry_msgs/Vector3`.|

### `ur16_repair`

Key components:

- **`repair_server.cpp`** – Action server binding robot control & perception.
    
- **`repair_operations.cpp`** – High‑level motion primitives (`moveHome`, scan trajectories, etc.).
    
- **Launch files**
    
    - `ur16_repair.launch.py` – One‑stop bring‑up, reading `robot_ip` & `target_frame` parameters.
        

> **Tip 🛠️** The C++ sources follow FIU‑ARC style guidelines and keep headers (`include/`) separate from implementation (`src/`).

---

## Docker (Work In Progress)

We are packaging the entire stack (ROS 2 Humble + UR driver + rmw_zenoh + ARC_RepairPlatform) into a single Docker image for **turn‑key deployment**:
[docker_container][https://github.com/scuar003/Semi_Autonomous_Repair]

---

## Media

Pictures, wiring diagrams, and demo videos will live in `docs/media/` and be embedded in this README. Stay tuned!

---

