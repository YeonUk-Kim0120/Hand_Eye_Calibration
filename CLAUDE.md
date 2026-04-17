# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 (ament_python) package for hand-eye calibration using a ZED 2i stereo camera and a UR5e robot arm. Supports both **eye-on-base** (camera fixed, checkerboard on end-effector) and **eye-in-hand** (camera on end-effector, checkerboard fixed) modes. Written in Korean and English; comments and README are predominantly Korean.

## Build & Run

The workspace root (`~/Desktop/Hand_Eye_Calibration`) is also the colcon workspace and the collector's output directory. The single ROS 2 package lives in `src/hand_eye_calibration/`.

```bash
# Build (from workspace root)
colcon build --symlink-install
source install/setup.bash

# Run nodes individually (each in a separate terminal, always source first)
ros2 run hand_eye_calibration detector          # checkerboard detection
ros2 run hand_eye_calibration tf_to_pose        # TF -> PoseStamped bridge
ros2 run hand_eye_calibration collector         # sample collection + calibration
ros2 run hand_eye_calibration mock_robot        # fake TF broadcaster for testing

# Collect a sample / run calibration (service calls)
ros2 service call /capture_sample std_srvs/srv/Empty
ros2 service call /run_calibration std_srvs/srv/Empty

# Lint (defined in package.xml test_depend)
ament_flake8
ament_pep257
```

The alternate Shah-method collector (`calibration_collector_node_sup.py`) is **not** registered in `setup.py`'s `console_scripts`, so it cannot be launched via `ros2 run`. Invoke it directly: `python3 src/hand_eye_calibration/hand_eye_calibration/calibration_collector_node_sup.py`.

## Architecture

The calibration pipeline is a three-node ROS 2 graph connected by topics and services:

```
ZED camera topics ──► [detector] ──/camera_to_checkerboard──►
                                                              [collector] ──► calibration result
UR5e TF tree ──► [tf_to_pose] ──/base_to_end_effector──────►
```

1. **`checkerboard_detector_node.py`** (`detector`) — Subscribes to ZED RGB image + CameraInfo via `message_filters.ApproximateTimeSynchronizer`. Runs OpenCV `findChessboardCorners` + `solvePnP`, publishes `PoseStamped` on `/camera_to_checkerboard`. Rate-limited via `processing_rate` param (default 2 Hz). Filters detections by reprojection error threshold.

2. **`tf_to_pose_publisher.py`** (`tf_to_pose`) — Bridges TF lookups (`parent_frame` -> `child_frame`, default `ur5e_base_link` -> `ur5e_tool0`) to `PoseStamped` on `/base_to_end_effector` at a configurable rate.

3. **`calibration_collector_node.py`** (`collector`) — Synchronizes the two PoseStamped topics. On `/capture_sample` service call, stores pose pair as 4x4 homogeneous matrices. On `/run_calibration`, calls `cv2.calibrateHandEye` (Tsai method). Uses `calibration_type` param (`eye_on_base` / `eye_in_hand`) to decide whether to invert T_base_ee before passing to the solver. Saves results to timestamped YAML and samples to `.npz`.

4. **`calibration_collector_node_sup.py`** — Alternate collector that uses `cv2.calibrateRobotWorldHandEye` (Shah method) instead. Eye-on-base only.

5. **`mock_robot_publisher_node.py`** (`mock_robot`) — Broadcasts a configurable TF (position/orientation set via ROS params at runtime) to simulate the robot for testing without hardware.

6. **`scripts/validate_detector.py`** — Standalone diagnostic tool that subscribes to `/camera_to_checkerboard` and prints stability/quality statistics every 5 seconds.

## Key Parameters

Declared as ROS 2 parameters — override with `--ros-args -p <name>:=<value>`.

- **`detector`:** `checkerboard_rows=6`, `checkerboard_cols=9`, `square_size=0.018` (meters), `pnp_method=IPPE` (also `ITERATIVE`, `SQPNP`), `max_reprojection_error=1.0` (pixels — detections above this are rejected), `processing_rate=2.0` (Hz), `show_visualization=True`.
- **`collector`:** `calibration_type=eye_on_base` (or `eye_in_hand`), `samples_file=calibration_samples.npz`, `auto_save=True`, `load_on_start=True`.
- **`tf_to_pose`:** `parent_frame=ur5e_base_link`, `child_frame=ur5e_tool0`, plus a publish rate.

## Key Conventions

- All transforms are stored and manipulated as 4x4 NumPy homogeneous matrices. Quaternion order is `(x, y, z, w)` everywhere (ROS convention).
- Pose-to-matrix conversion uses `tf_transformations.quaternion_matrix` + `translation_matrix` composed as `T = T_trans @ T_rot`.
- The collector hardcodes its working directory to `~/Desktop/Hand_Eye_Calibration` for output files. It **auto-loads `calibration_samples.npz` on startup** (`load_on_start=True`) — to begin a fresh session, delete or rename the `.npz` file, or pass `-p load_on_start:=false`.
- Image/info topics are hardcoded to `/zed/zed_node/rgb/image_rect_color` and `/zed/zed_node/rgb/camera_info`; the detector also hardcodes `camera_frame_id="zed_left_camera_optical_frame"` on its published `PoseStamped`.
- TF frame names default to `ur5e_base_link` and `ur5e_tool0` — these vary by UR robot configuration.

## Dependencies

- **System:** ROS 2 (Humble or later), ZED SDK, ZED ROS 2 Wrapper (separate workspace)
- **Python:** `numpy==1.26.4`, `opencv-python`, `pyyaml`, `transforms3d`, `tf_transformations` (from git)
- **ROS packages:** `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_srvs`, `cv_bridge`, `message_filters`, `tf2_ros`
