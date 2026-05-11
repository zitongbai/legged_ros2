# Installation Guide for Legged ROS2

This document provides step-by-step instructions to install and set up the Legged ROS2 package for controlling, simulating, and deploying legged robots.

[TOC]


## Local Run

### Environment

- Ubuntu 22.04 LTS
- ROS 2 Humble

### Prerequisites

Before installing Legged ROS2, make sure the following prerequisites are ready:

1. **ROS 2 Humble**: Follow the official ROS 2 installation guide for Humble [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
2. **ROS 2 Control**: Install ROS 2 Control packages:
   ```bash
   sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
   ```
3. **Unitree ROS 2**: Download and build Unitree ROS 2 by following the instructions in the [Unitree ROS 2 repository](https://github.com/unitreerobotics/unitree_ros2).
   - Remember to modify `setup.sh` (for real robot) or `setup_local.sh` (for simulation) for your setup, including ROS distribution, path to `unitree_ros2/cyclonedds_ws/install/setup.bash`, and network configuration.
   - Many operations later will require sourcing either `setup.sh` or `setup_local.sh` in `unitree_ros2`.

### Installation

1. **Clone the repository**:
   ```bash
   cd ~/legged_ws/src
   git clone https://github.com/zitongbai/legged_ros2.git
   ```
2. **Download ONNX Runtime** to the `third_party` directory:
   ```bash
   cd ~/legged_ws/src/legged_ros2/third_party
   wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
   tar -xvzf onnxruntime-linux-x64-1.22.0.tgz
   ```
3. **Install dependencies** with `rosdep`:
   ```bash
   cd ~/legged_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```
4. **Build the workspace** with `colcon`:
   ```bash
   source path/to/your/unitree_ros2/setup.sh  # or setup_local.sh for simulation
   colcon build --symlink-install
   ```

After installation, continue with the [Usage Guide](usage.md).

## Docker Run

If you prefer to use Docker for installation and running Legged ROS2, follow the instructions below.

### Build a production image

From the `legged_ros2` repository root, build the production image with the helper script:

```bash
docker/build.sh
```

### Start and initialize the container

Run the container setup script from the `legged_ros2` repository root:

```bash
docker/run.sh
```

`docker/run.sh` starts the production container, mounts the local `legged_ros2` repository to `/root/legged_ws/src/legged_ros2`, and performs the one-time workspace initialization inside the container:

- download ONNX Runtime into `third_party` if it is missing
- install package dependencies with `rosdep`
- build the workspace with `colcon build --symlink-install` if `install/setup.bash` is not present

If a container with the same name already exists, `run.sh` removes it first and creates a fresh one. After initialization finishes, `run.sh` opens an interactive shell inside the container in the current terminal.

### Enter the running container

Use `docker/enter.sh` when you want to open another terminal in the already running container:

```bash
docker/enter.sh
```

`docker/enter.sh` does not create or initialize the container. It only runs `/bin/bash` inside the existing `legged-ros2-humble` container.

Then choose one setup script based on your use case:

```bash
source /root/legged_ws/setup.sh
# or
source /root/legged_ws/setup_local.sh
```

## What's Next?

After installation, continue with the [Usage Guide](usage.md).
