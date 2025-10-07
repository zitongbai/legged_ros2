# legged_ros2

[![Ubuntu 20.04/22.04](https://img.shields.io/badge/Ubuntu-22.04-blue.svg?logo=ubuntu)](https://ubuntu.com/) [![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg?logo=ros)](https://docs.ros.org/en/humble/index.html)

ROS 2 packages for the control, simulation, and deployment of legged robots. Mujoco simulation can be performed in two ways:

- **Mujoco Direct**: Implements a custom hardware interface that communicates directly with Mujoco via its API. This approach offers greater flexibility and supports non-Unitree robots.
- **Mujoco SDK2**: Uses the modified Mujoco version provided by Unitree, allowing the same hardware interface as the real robot through the Unitree SDK2. This enables running identical code on both the real robot and the Mujoco simulation, providing a more convenient workflow for Unitree robots.

The deployment is based on the [ROS2 Control](https://control.ros.org/humble/doc/getting_started/getting_started.html) framework. 

Supported robots:

| Robot Name  |   Mujoco Direct    |    Mujoco SDK2     |        Real        |
| :---------: | :----------------: | :----------------: | :----------------: |
| Unitree G1  | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |
| Unitree Go2 | :heavy_check_mark: | :heavy_check_mark: |        TODO        |


## Installation

Please refer to the [installation guide](docs/installation.md).

## Usage

Use Unitree G1 as an example, the usage is similar for other robots.

### Mujoco Direct

Todo: Add usage instructions for Mujoco Direct.

### Mujoco SDK2

> A **joystick** (support "xbox" and "switch" gamepad layout) connected to your computer is required to control the robot in the Mujoco simulation.

1. Launch the Mujoco simulation in one terminal:

```bash
source ~/legged_ws/install/setup.bash
ros2 launch unitree_mujoco sim.launch.py config:=config_g1.yaml
```

The config file is located at: `~/legged_ws/src/legged_ros2/third_party/unitree_mujoco/simulate/config/config_g1.yaml`. For more details about the config's parameters, please refer to [Unitree Mujoco](https://github.com/unitreerobotics/unitree_mujoco?tab=readme-ov-file#c-simulator).

Press '9' to activate or release the strap, press '8' to lower the robot, and press '7' to lift the robot.

2. Launch the controllers in another terminal:

```bash
source ~/legged_ws/install/setup.bash
ros2 launch g1_description sdk.launch.py
```

Args of `sdk.launch.py`:
- `gui`: Launch the rviz2 to visualize the robot state, default is `false`.
- `network_interface`: Specify the network interface used for Unitree SDK2 communication, default is `lo`, which is used for simulation.

3. You would see a ros controller manager GUI pop up, which shows the state of the controllers. Joystick Operation Instructions:
- A: Switch to RL controller
- B: Switch to Static controller
- Y: Switch to Joint State Broadcaster only
- left joystick axis: Move the robot forward/backward and left/right
- right joystick axis: Rotate the robot

### Real Robot

> [!CAUTION]
> **Users are advised to follow Unitree's recommendations for safety while using the real robot. This is research code; use at your own risk; we do not take responsibility for any damage.**

1. Setup the real robot according to [Unitree's official documentation](https://support.unitree.com/home/en/G1_developer/quick_development).

2. Launch the controllers:
   
```bash
source ~/legged_ws/install/setup.bash
ros2 launch g1_description sdk.launch.py network_interface:=<your_network_interface>
```

3. Operate the robot using the joystick as in simulation. 

## Sim2Sim / Sim2Real Tips

* Order of joints
* `armature` of joint in mujoco model (refer to [armature in mujoco doc](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint-armature))
* IMU mounting position and orientation
* Joint limits (RL policy might exploit the joint limits, if the real robot has different joint limits, the policy might not work well)


