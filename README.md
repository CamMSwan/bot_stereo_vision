<p align="center">
  <a href="https://botbot.bot" target="_blank">
    <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/672ed83e9ab7d55f18a3c43f_BotBot%20Purple%20Logo%20(2)-p-500.png" alt="BotBot" width="180">
  </a>
</p>

# bot_stereo_vision - Low Cost Stereo Vision Module

ROS 2 package for low-cost stereo vision-based 3D perception, developed as a Capstone project (TCC) for the Engineering program at Insper during the 2026.1 semester. The system reconstructs the three-dimensional environment and generates hardware-accelerated depth maps directly on the GPU, providing crucial perception data for the autonomous navigation and obstacle avoidance of the **BotBot** robot.

## Purpose

The `bot_stereo_vision` package serves as a low-cost alternative to commercial depth sensors (such as LiDAR sensors and dedicated RGB-D cameras). It manages:
- Synchronized dual-camera stereo frame capture via V4L2/OpenCV
- GPU-accelerated deep stereo matching using FastFoundation-Stereo
- Dense depth map generation
- ROS 2 topic publishing compatible with the BotBrain localization and navigation stacks

## Requirements

For the artificial intelligence pipeline and stereo processing to run in real-time, the hardware and software environment must strictly follow the specifications below.

### 1. System Specifications

- **Target Hardware:** NVIDIA Jetson Orin Nano (Developer Kit or custom carrier board)
- **Sensors:** 2x or 4x Logitech C920 USB cameras mounted on one or two rigid stereo bases
- **Operating System:** Ubuntu 22.04 LTS with **JetPack 6.x** (L4T 36.5.0)
- **Middleware:** ROS 2 Humble Hawksbill (Standard installation via `apt`)

### 2. Environment Dependency Matrix (Python/CUDA)

Due to ARM64 architecture constraints and hardware acceleration buses, installing AI dependencies via the standard global Python repository (`PyPI`) installs CPU-only packages, rendering the project unviable.

The environment must be stabilized using the following version matrix:

- **PyTorch:** `2.3.0` (Officially compiled by NVIDIA with CUDA support)
- **Torchvision:** `0.18.0` (Natively linked to CUDA kernels)
- **OpenAI Triton:** `3.5.0` (Required for local compilation of Group-wise Correlation — GWC volumes)
- **NumPy:** `>=1.21.5, <2.0` (Locked to the 1.x tree to maintain compatibility with ROS `cv_bridge` and prevent C-API breakages)
- **OpenCV Python:** `4.9.0.80` (Version compatible with the NumPy 1.x tree)
- **TensorRT:** `10.3.0` (Injected via native JetPack packages)

### 3. Automated Hardware Preparation

Before building the ROS 2 workspace, hardware dependencies must be injected locally. The script included in the `requirements/` folder cleanly automates the entire process (cleaning CPU remnants, downloading 1.1GB wheels from NVIDIA, and Cargo/Pip optimization).

```bash
cd /bot_stereo_vision/requirements/
chmod +x setup_jetson.sh
./setup_jetson.sh
```

### 4. Model Installation

Create a `model/` folder inside the main structure and import the FastFoundation-Stereo model into it.

### 5. Camera Calibration


## Package Files

### Launch Files

---

#### `launch/C920_cameras.launch.py`

Master launch file to initialize and orchestrate the multi-camera low-cost stereo vision pipeline.

**Description**: Master launch file that orchestrates the entire stereo perception and depth processing pipeline for the BotBot robot platform. It initializes the main multi-camera orchestration node, sets up the physical and optical static coordinate transform chains (TFs) for both sensor rigs, and brings up conversion nodes to map raw depth data into standard 2D laser scans for autonomous navigation support.

**What Gets Launched**:

This launch file brings up and manages the following nodes simultaneously:

1. **stereo_vision_orchestrator** (`bot_stereo_vision/stereo_node`): The primary perception hub handling front and rear stereo camera streams concurrently, pulling configurations from independent YAML parameters and executing the stereo pipeline on the GPU.
2. **tf_front_phys** (`tf2_ros/static_transform_publisher`): Establishes the static spatial transform between the robot's base frame (`base_link`) and the front stereo rig's physical center reference (`front_camera_link`).
3. **tf_front_opt** (`tf2_ros/static_transform_publisher`): Maps the standard 3D optical coordinate alignment (`front_camera_optical_link`) relative to the physical front frame to align computer vision data with ROS conventions.
4. **tf_back_phys** (`tf2_ros/static_transform_publisher`): Maps the rear stereo rig frame (`back_camera_link`) relative to `base_link`, applying a 180° (π rad) yaw rotation to properly orient the sensors backward.
5. **tf_back_opt** (`tf2_ros/static_transform_publisher`): Maps the standard 3D optical coordinate alignment (`back_camera_optical_link`) relative to the physical rear frame.
6. **depthimage_to_laserscan_front** (`depthimage_to_laserscan/depthimage_to_laserscan_node`): Converts the forward rectified depth image stream into a standard 2D laser scan format for forward obstacle avoidance.
7. **depthimage_to_laserscan_back** (`depthimage_to_laserscan/depthimage_to_laserscan_node`): Converts the rearward rectified depth image stream into a standard 2D laser scan format for rearward obstacle avoidance.

**Parameters**:

| Node / Action | Parameter | Type | Default / Value | Description |
| :--- | :--- | :--- | :--- | :--- |
| `stereo_vision_orchestrator` | `config_front` | `string (path)` | `config/front_camera.yaml` | YAML configuration file path containing intrinsic parameters and topic configurations for the front camera rig. |
| `stereo_vision_orchestrator` | `config_back` | `string (path)` | `config/back_camera.yaml` | YAML configuration file path containing intrinsic parameters and topic configurations for the rear camera rig. |
| `stereo_vision_orchestrator` | `enabled_cameras` | `list` | `['front', 'back']` | Defines which stereo rigs are active during initialization (allows toggling single or multi-rig execution). |
| `depthimage_to_laserscan_*` | `range_max` | `double` | `5.0` | Maximum tracking range in meters for the generated 2D planar laser scan. |
| `depthimage_to_laserscan_*` | `range_min` | `double` | `0.3` | Minimum blind-zone tracking range in meters for the generated 2D laser scan. |
| `depthimage_to_laserscan_*` | `scan_height` | `int` | `3` | Height of the pixel rows (in pixels) sampled from the depth image to generate the virtual scanline. |

### Scripts

---

#### `scripts/frame_grabber_stereo.py`

Threaded dual-camera synchronized frame capture utility handling low-level V4L2 driver modifications and real-time OpenCV geometric image rectification.

**Functionality**:
- **Hardware Parameter Locking**: Spawns automated `v4l2-ctl` system subprocesses to apply manual image controls (exposure limit, gain, contrast, brightness, and static white balance) directly to `/dev/videoX` devices, eliminating auto-exposure synchronization mismatches between the twin lenses.
- **Threaded Sensor Polling**: Orchestrates an asynchronous, non-blocking background thread loop that captures simultaneous left and right hardware video frames to prevent USB bus starvation and optimize capture frame rates.
- **Epipolar Rectification**: Utilizes pre-compiled camera calibration XML lookup arrays to perform mapping operations on raw streams, instantly flattening lens barrel distortions and forcing perfect horizontal scanline alignment.
- **Feature Enhancement & Padding**: Applies CLAHE (Contrast Limited Adaptive Histogram Equalization) to mitigate lighting reflection clipping, downscales the resolution profile to match network inputs, and injects bounding edge padding to meet strict network geometry constraints (multiples of 32 pixels).

---

#### `scripts/depth_processor.py`

GPU-accelerated deep stereo inference engine integrating FastFoundation-Stereo with TensorRT for real-time dense disparity and depth map generation.

**Functionality**:
- **GPU Tensor Orchestration**: Converts unwarped Left and Right NumPy matrix pairs into high-dimensional PyTorch CUDA float tensors, reordering channel dimensions (`NCHW`) for optimized GPU pipeline consumption.
- **Depth Post-Processing**: Strips out temporary network padding dimensions from the output disparity slices, clips mathematical edge anomalies, and applies rigid distance cutoff boundaries (`min_depth` and `max_depth`) to filter reflection noise out of the downstream navigation costmaps.

### Configuration Files

---

#### `config/front_camera.yaml` & `config/back_camera.yaml`

Sensor deployment configurations containing intrinsic geometry, V4L2 hardware parameters, and hardware device indexing for both front and rear stereo rigs.

**Description**: These configuration files store runtime parameters for the stereo capture nodes. They define critical lens physics dimensions (baseline, depth range thresholds) and directly inject driver-level V4L2 parameters to lock exposure, contrast, and gain manually. Locking these settings eliminates auto-exposure mismatches between the independent left and right USB streams, guaranteeing consistent visual input for the deep stereo matching pipeline.

**Parameters**:

| Parameter | Type | Default / Value | Description |
| :--- | :--- | :--- | :--- |
| `position` | `string` | `"front"` / `"back"` | Physical placement marker indicating rig orientation on the chassis. |
| `baseline` | `double` | `0.0976` | Physical distance in meters between the optical centers of the left and right lenses. |
| `focal_length` | `double` | `500.0` | Baseline estimate for pixel focal length, runtime auto-adjusted by camera calibration arrays. |
| `min_depth` | `double` | `0.2` | Minimum reliable depth estimation boundary threshold in meters. |
| `max_depth` | `double` | `5.5` | Maximum depth estimation filtering boundary threshold in meters. |
| `cam_right_id` | `int` | `0` (front) / `4` (back) | Linux Video4Linux engine index (`/dev/videoX`) assigned to the right sensor stream. |
| `cam_left_id` | `int` | `2` (front) / `6` (back) | Linux Video4Linux engine index (`/dev/videoX`) assigned to the left sensor stream. |
| `width` | `int` | `640` | Native sensor horizontal resolution capture profile in pixels. |
| `height` | `int` | `480` | Native sensor vertical resolution capture profile in pixels. |
| `fps` | `double` | `10.0` | Target frame capture rate constraint. |
| `target_width` | `int` | `160` | Downscaled network input width resolution for real-time TensorRT acceleration. |
| `target_height` | `int` | `120` | Downscaled network input height resolution for real-time TensorRT acceleration. |
| `auto_exposure` | `int` | `1` | Exposure control policy flag (`1`: Manual lock, `3`: Aperture Priority Mode). |
| `exposure_time_absolute` | `int` | `39` | Rigid sensor electronic shutter time limit (lower values mitigate motion blur artifacts). |
| `exposure_dynamic_framerate` | `int` | `0` | Disables driver-side framerate drops under low-light operating constraints. |
| `gain` | `int` | `10` | Static sensor pre-amplifier analog signal multiplier. |
| `brightness` | `int` | `128` | Static black level offset calibration point. |
| `contrast` | `int` | `128` | Locked dynamic range slope parameter to minimize high-intensity reflection clipping. |
| `saturation` | `int` | `128` | Locked color amplitude profile setting. |
| `backlight_compensation` | `int` | `0` | Disables driver-level automatic exposure adjustments for backlighting. |
| `power_line_frequency` | `int` | `0` | Artificial lighting anti-flicker filter configuration (`0`: Disabled, `1`: 50Hz, `2`: 60Hz). |
| `white_balance_automatic` | `int` | `0` | Disables independent dynamic white balance to prevent color-space shift anomalies between feeds. |

---

#### `config/stereo_map_front.xml` & `config/stereo_map_back.xml`

Pre-computed OpenCV geometric rectification lookup tables for lens distortion unwarping and epipolar alignment.

**Description**: These XML documents contain the storage matrices generated from a rigorous checkerboard calibration sequence. They map high-precision lookup coordinate variables (`stereoMapL_x`, `stereoMapL_y`, `stereoMapR_x`, `stereoMapR_y`) along with the fundamental projection parameters (`Q`, `R`, `T`). The frame grabber executes a high-speed matrix remapping operation using these files to flatten barrel distortions and bring left and right scan lines into perfect horizontal epipolar alignment prior to cost volume initialization.

## Topics

### Published
- `/{pos}_camera/color/image_raw` (`sensor_msgs/msg/Image`): Raw rectified left-lens visual stream matching network alignment frames.
- `/{pos}_camera/color/camera_info` (`sensor_msgs/msg/CameraInfo`): Intrinsic projection properties scaled for the visual image frame.
- `/{pos}_camera/aligned_depth_to_color/image_raw` (`sensor_msgs/msg/Image`): Dense 16-bit depth matrix scaled in millimeters, mapped directly to visual channel spaces for RTAB-Map SLAM tracking.
- `/{pos}_camera/depth/image_rect_raw` (`sensor_msgs/msg/Image`): Dense rectified depth image channel allocated for planar navigation conversions.
- `/{pos}_camera/depth/camera_info` (`sensor_msgs/msg/CameraInfo`): Coordinate frame and focal properties mirroring the depth matrix channel layer.

**Note**: `{pos}` is a dynamic namespace variable resolved at runtime based on entries declared in the `enabled_cameras` configuration parameter array (e.g., generating topics under `/front_camera/...` and `/back_camera/...` simultaneously).

## Mapping Workflow

### Launching the System

```bash
ros2 launch bot_stereo_vision C920_cameras.launch.py
```

## Directory Structure

A organização do repositório foi projetada para manter o pacote do ROS 2 completamente isolado de scripts de pareamento de ambiente externo, garantindo portabilidade entre workspaces.

```text
bot_stereo_vision/                  # Package root (must be cloned inside botbrain_ws/src/)
├── bot_stereo_vision/              # Main source code directory of the Python module
│   ├── __init__.py                 # Python package initializer
│   └── stereo_vision_node.py       # Main ROS 2 node (Stereo Pipeline Orchestrator)
├── config/                         # Calibration and initialization parameters for the sensors
│   ├── back_camera.yaml            # Topic/parameter configuration for the rear camera
│   ├── front_camera.yaml           # Topic/parameter configuration for the front camera
│   ├── stereo_map_back.xml         # Intrinsic/extrinsic rectification matrices (Rear Camera)
│   └── stereo_map_front.xml        # Intrinsic/extrinsic rectification matrices (Front Camera)
├── launch/                         # Node initialization automation scripts
│   └── C920_cameras.launch.py      # Official launch file that initializes the vision pipeline
├── model/                          # Directory reserved for the Fast-FoundationStereo model (Must be created by the user)
├── requirements/                   # Infrastructure and environment governance external to ROS
│   ├── requirements.txt            # List of dependencies required to run the project
│   └── setup_jetson.sh             # Bash script for clean injection of NVIDIA CUDA packages and installing dependencies
├── resource/                       # ROS ecosystem indexing file
│   └── bot_stereo_vision           # Index registry for package discovery by 'ament'
├── scripts/                        # Helper classes and pure computer vision algorithms
│   ├── __init__.py                 # Allows local module importation
│   ├── depth_processor.py          # Processor and generator of the correlation and depth map
│   └── frame_grabber_stereo.py     # Synchronized frame capture via V4L2/OpenCV
├── .gitignore                      # Filter for disposable files for version control (Git)
├── package.xml                     # Package metadata and ROS 2 dependency declarations
├── README.md                       # Official technical documentation of the system
├── setup.cfg                       # Configuration of installation directories for setuptools
└── setup.py                        # Colcon build script and console_scripts mapping
```
<p align="center">Made in Brazil</p>

<p align="right">
  <img src="https://cdn.worldvectorlogo.com/logos/insper.svg" alt="Insper" height="60">
  &nbsp;&nbsp;
  <img src="https://cdn.prod.website-files.com/672ed723fbdc1589fa127239/67522c0342667cac3a16a994_Bot%20icon%20(1).png" alt="Bot icon" width="110">
</p>