# 🚀 Installation 


Create a ros2 workspace and clone the repository:

```bash
mkdir -p ~/legged_ws/src
cd ~/legged_ws/src
# Https: 
git clone https://github.com/zitongbai/legged_ros2.git
# or SSH:
# git clone git@github.com:zitongbai/legged_ros2.git
```

Before building `legged_ros2`, make sure you have installed the following dependencies.

## 🛠️ Dependencies

✅ **ROS2 Control and ROS2 Controllers** 

The framework is based on the ROS2 Control, install the core packages:

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

🔧 **Mujoco** 

Install some dependencies:

```bash
sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev
```

Download **Mujoco 3.2.7** from the [official repository](https://github.com/google-deepmind/mujoco/releases). Extract the downloaded file and place it in any directory you prefer. For example, you can extract it to your home directory. Then set the environment variable:

```bash
# you can add this command to your ~/.bashrc file
export MUJOCO_DIR=/PATH/TO/mujoco-3.2.7
```

📦 **ONNX Runtime**

We use ONNX Runtime to run the learned policy. Place the runtime under `src/legged_ros2/third_party`:


```bash
cd ~/legged_ws/src/legged_ros2/third_party
# Download the ONNX Runtime
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
# Extract the downloaded file
tar -xvzf onnxruntime-linux-x64-1.22.0.tgz
```
<details>
<summary><strong>🤖 Unitree SDK2 (Optional)</strong></summary>

If you want to simulate Unitree robots (e.g. Go2, G1) in Mujoco via the Unitree SDK2, or deploy the controller to a real Unitree robot, you need to install the Unitree SDK2.

Please follow the instructions in the [Unitree official repository](https://github.com/unitreerobotics/unitree_sdk2) to install the Unitree SDK2. 

> **Important:**  
> Make sure to install the SDK2 to the specific path by setting the `CMAKE_INSTALL_PREFIX` to `/opt/unitree_robotics` when building the SDK2.

</details>

<details>
<summary><strong>🔁 Change DDS Implementation (Optional)</strong></summary>

If you want to use the **Mujoco SDK2** method to simulate the Unitree robots, you need to change the default DDS implementation, otherwise you may encounter error, see this [issue](https://github.com/unitreerobotics/unitree_mujoco/issues/46). The following steps is referred from [unitree official doc](https://support.unitree.com/home/en/G1_developer/ros2_communication_routine).



First install the dependences:
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
```

Before compiling cyclonedds, please ensure that ros2 environment has NOT been sourced when starting the terminal. Otherwise, it may cause errors in compilation.

If `source/opt/ros/humble/setup.bash` has been added to the `~/.bashrc` file when installing ROS2, it needs to be commented out:

```bash
sudo apt install gedit
sudo gedit ~/.bashrc
```

```bash
# in .bashrc
# source /opt/ros/humble/setup.bash 
```

Then build cyclone-dds:

```bash
mkdir -p ~/cyclonedds_ws/src
cd ~/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd ..
colcon build --packages-select cyclonedds #Compile cyclone-dds package
source /opt/ros/humble/setup.bash # source ROS2 environment
colcon build # Compile all packages in the workspace
```

After that, you can rewrite the `source/opt/ros/humble/setup.bash` in `.bashrc`:
```bash
# in .bashrc
source /opt/ros/humble/setup.bash
source ~/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# To avoid conflict with Unitree SDK2's DDS Communication:
export ROS_DOMAIN_ID=10 # or other number you like, except 0
```
</details>


## 🏗️ Build

Now you can build the workspace:

```bash
cd ~/legged_ws
colcon build --symlink-install
```
if you want to simulate Unitree robots:
```bash
colcon build --symlink-install --cmake-args -DBUILD_UNITREE=ON
```
