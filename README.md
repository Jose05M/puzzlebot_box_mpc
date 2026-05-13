````md
# Puzzlebot Visual Servoing with Sampled MPC

This project implements a vision-based autonomous navigation system for the Puzzlebot platform using a calibrated Sampled Model Predictive Controller (MPC).

The robot detects colored boxes using a monocular RGB camera, estimates relative distance and angular error directly from image features, and autonomously approaches the target using visual servoing.

After reaching the desired distance, the robot navigates toward a delivery waypoint and finally returns to its home position using odometry-based control.

---

# System Overview

The system combines:

- Visual perception using OpenCV
- Relative state estimation $(\rho,\alpha)$
- Sampled Model Predictive Control (MPC)
- Differential-drive odometry
- Finite State Machine (FSM)
- Waypoint and return-home navigation

Main mission flow:

```text
WAIT_COMMAND
    ↓
TRACK_TARGET (Visual Servoing + MPC)
    ↓
COLLECTING
    ↓
GO_TO_WAYPOINT
    ↓
RETURN_HOME
    ↓
WAIT_COMMAND
````

---

# Repository Structure

```text
.
├── teleop.py
├── puzzlebot_odometry.py
├── mpc_hw.py
├── calibration_node.py
├── analyze_calibration.py
├── mpc_results.csv
└── README.md
```

## Files Description

### `teleop.py`

Keyboard interface used to send commands to the robot:

* select target color,
* send waypoint commands,
* return home,
* cancel missions.

---

### `puzzlebot_odometry.py`

ROS2 node that computes differential-drive odometry using wheel encoder velocities.

Publishes:

```text
/odom
```

---

### `mpc_hw.py`

Main node of the project.

Implements:

* color detection,
* target selection,
* distance and angle estimation,
* Sampled MPC,
* finite state machine,
* waypoint navigation,
* return-home controller,
* CSV logging.

---

### `calibration_node.py`

Used to generate experimental data for model calibration.

Performs:

* linear motion trials,
* angular motion trials,
* focal calibration,
* visual data logging.

---

### `analyze_calibration.py`

Offline analysis script used to estimate:

* $K_\rho$
* $K_\alpha$

for the predictive MPC model.

---

# Dependencies

## ROS2

Tested on:

```text
ROS2 Humble
```

---

## Python Libraries

Install required packages:

```bash
pip install opencv-python numpy pandas transforms3d
```

---

# ROS2 Topics

## Subscribed Topics

| Topic                | Type                | Description       |
| -------------------- | ------------------- | ----------------- |
| `/video_source/raw`  | `sensor_msgs/Image` | RGB camera stream |
| `/odom`              | `nav_msgs/Odometry` | Robot odometry    |
| `/box_color_command` | `std_msgs/String`   | User commands     |

---

## Published Topics

| Topic      | Type                  | Description          |
| ---------- | --------------------- | -------------------- |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands    |
| `/odom`    | `nav_msgs/Odometry`   | Estimated robot pose |

---

# How to Run

---

## 1. Run Odometry Node

```bash
python3 puzzlebot_odometry.py
```

This node estimates the robot pose from wheel encoder velocities.

---

## 2. Run Main MPC Node

```bash
python3 mpc_hw.py
```

This launches:

* visual perception,
* MPC controller,
* FSM,
* waypoint navigation,
* logging system.

---

## 3. Run Teleoperation Commands

```bash
python3 teleop.py
```

This opens the keyboard interface for mission commands.

---

# Teleoperation Commands

| Key     | Action           |
| ------- | ---------------- |
| `g`     | Track green box  |
| `p`     | Track pink box   |
| `y`     | Track yellow box |
| `h`     | Return home      |
| `w x y` | Go to waypoint   |
| `c`     | Cancel mission   |
| `q`     | Exit             |

Example:

```text
w 1.5 2.0
```

sends the robot to waypoint:

```text
(1.5 , 2.0)
```

---

# MPC Predictive Model

The controller operates on the relative visual state:

$$
x = [\rho,\alpha]^T
$$

where:

* $\rho$ = estimated distance to target,
* $\alpha$ = relative angular error.

The calibrated predictive model is:

$$
\rho_{k+1} =
\rho_k -
K_{\rho} v_k \cos(\alpha_k)\Delta t
$$

$$
\alpha_{k+1} =
\alpha_k -
K_{\alpha}\omega_k\Delta t
$$

The control inputs are:

$$
u = [v,\omega]^T
$$

The controller evaluates sampled control candidates over a finite prediction horizon and selects the action with minimum cost.

---

# Calibration Procedure

The MPC model parameters were experimentally calibrated using controlled motion experiments.

---

## 1. Run Calibration Node

```bash
python3 calibration_node.py
```

---

## 2. Execute Calibration Trials

Keyboard commands:

| Key | Action                  |
| --- | ----------------------- |
| `k` | Calibrate focal length  |
| `a` | Positive angular motion |
| `d` | Negative angular motion |
| `v` | Linear forward motion   |
| `x` | Stop robot              |

---

## 3. Generate Calibration Data

The node saves:

```text
calibration_data.csv
```

---

## 4. Analyze Calibration Data

```bash
python3 analyze_calibration.py
```

This estimates:

* $K_\rho$
* $K_\alpha$

used by the predictive model.

---

# Logged Experimental Data

During execution, the MPC node stores experimental data in:

```text
mpc_results.csv
```

The file includes:

| Variable          | Description        |
| ----------------- | ------------------ |
| `rho`             | Estimated distance |
| `alpha`           | Angular error      |
| `v_cmd`           | Linear velocity    |
| `w_cmd`           | Angular velocity   |
| `cost`            | MPC cost           |
| `x,y,theta`       | Odometry pose      |
| `state`           | FSM state          |
| `target_detected` | Detection flag     |

---

# Results

The system was experimentally validated on a real Puzzlebot platform.

The robot successfully performed:

* visual target tracking,
* autonomous approach,
* waypoint navigation,
* return-home behavior,
* multiple mission cycles.

The implementation also includes:

* smooth control actions,
* bounded velocity commands,
* FSM-based mission coordination,
* experimental data logging.

---

# Example Trajectory

*Add your trajectory image here*

```md
![Trajectory](01_trajectory_xy.png)
```

---

# Future Improvements

Possible future extensions include:

* obstacle avoidance,
* dynamic waypoint generation,
* full IBVS interaction matrix,
* SLAM integration,
* manipulator integration,
* nonlinear MPC,
* multi-object mission planning.

---

# Report

This repository contains only the implementation and execution pipeline.

For detailed:

* mathematical derivations,
* controller formulation,
* experimental analysis,
* system modeling,

refer to the full project report.

---

# Author

José Eduardo Sánchez Martínez

```
```
