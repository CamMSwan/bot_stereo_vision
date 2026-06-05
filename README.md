# Lost Cost Stereo Vision Module (`bot_stereo_vision`)

**CAPSTONE - Insper (2026.1 Semester)**

---

## Purpose

This repository contains the `bot_stereo_vision` ROS 2 package, developed as a capstone project (TCC) for the Engineering program at Insper during the 2026.1 semester. 

The core objective of the project is to design, implement, and validate a low-cost deep stereo vision system (utilizing two conventional Logitech C920 cameras) capable of acting as an accessible and efficient alternative to commercial depth sensors (such as LiDAR sensors and dedicated RGB-D cameras). The system reconstructs the three-dimensional environment and generates hardware-accelerated depth maps directly on the GPU, providing crucial perception data for the autonomous navigation and obstacle avoidance of the **BotBot** robot.

---

## Requirements

For the artificial intelligence pipeline and stereo processing to run in real-time, the hardware and software environment must strictly follow the specifications below.

### 1. System Specifications

* **Target Hardware:** NVIDIA Jetson Orin Nano (Developer Kit or custom carrier board).
* **Sensors:** 2x or 4x Logitech C920 USB cameras mounted on one or two rigid stereo bases.
* **Operating System:** Ubuntu 22.04 LTS with **JetPack 6.x** (L4T 36.5.0).
* **Middleware:** ROS 2 Humble Hawksbill (Standard installation via `apt`).

### 2. Environment Dependency Matrix (Python/CUDA)

Due to ARM64 architecture constraints and hardware acceleration buses, installing AI dependencies via the standard global Python repository (`PyPI`) installs CPU-only packages, rendering the project unviable.

The environment must be stabilized using the following version matrix:

* **PyTorch:** `2.3.0` (Officially compiled by NVIDIA with CUDA support).
* **Torchvision:** `0.18.0` (Natively linked to CUDA kernels).
* **OpenAI Triton:** `3.5.0` (Required for local compilation of Group-wise Correlation — GWC volumes).
* **NumPy:** `>=1.21.5, <2.0` (Locked to the 1.x tree to maintain compatibility with ROS `cv_bridge` and prevent C-API breakages).
* **OpenCV Python:** `4.9.0.80` (Version compatible with the NumPy 1.x tree).
* **TensorRT:** `10.3.0` (Injected via native JetPack packages).

### 3. Automated Hardware Preparation

Before building the ROS 2 workspace, hardware dependencies must be injected locally. The script included in the `requirements/` folder cleanly automates the entire process (cleaning CPU remnants, downloading 1.1GB wheels from NVIDIA, and Cargo/Pip optimization).

To run the automation, execute the following in the Jetson terminal:

```bash
cd /bot_stereo_vision/requirements/
chmod +x setup_jetson.sh
./setup_jetson.sh
```
### 4. Model Installation
Now it is necessary to create a `model/` folder inside the main structure, where you will import the FastFoundation-Stereo model.


---

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
