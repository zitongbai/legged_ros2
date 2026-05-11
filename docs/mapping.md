# Legged Mapping

Package `legged_mapping` provides tools to work with maps in legged robot applications. It does not perform mapping itself but offers utilities to facilitate the use of maps.

Here we provide a guide to set up mapping using the FAST-LIO 2 SLAM algorithm with legged robots equipped with LiDAR sensors.

The package currently provides a configurable static TF helper node. It is intended to complement an external LIO stack, not replace it.

Most of the following operations are carried out **on PC instead of the robot's onboard computer**, in order to have better performance and avoid conflicts with other processes running on the robot.

Currently, only [Unitree Go2 with MID360](https://support.unitree.com/home/en/developer/SLAM%20and%20Navigation_service) is supported, but support for additional robots and sensors will be added in the future.

- [x] Unitree Go2 with MID360
- [ ] Unitree G1 with MID360
- [ ] Unitree Go2 with XT16

## Guide for Mapping with FAST-LIO 2

### Unitree Go2 with MID360

This section provides instructions for setting up mapping on the Unitree Go2 robot equipped with the MID360 LiDAR sensor using the FAST-LIO 2 SLAM algorithm.

#### Hardware Connections

Mount the MID360 LiDAR sensor on the Unitree Go2 robot and connect it to the robot's onboard computer via specified cable. Then connect your PC to the robot's onboard computer through Ethernet. 

```mermaid
graph LR;
    A[PC] <-- Ethernet --> B[Go2];
    B <-- Cable --> C[MID360];

```

Because the communication with MID360 is via network, we can directly get data from MID360 and modify the settings of the MID360 via Livox Viewer 2 software in the PC.

#### Prerequisites

1. If you use the MID360 provided by Unitree, you can skip this step because it is already configured to work with the Go2 robot. Otherwise, you need to set up the MID360 LiDAR sensor first. Use the Livox Viewer 2 software and modify the network settings of the MID360 as follows:
    ```
    Lidar IP: 192.168.123.20
    Gateway Address: 192.168.123.1
    Lidar Info IP: 192.168.123.70
    ```
2. Install `Livox-SDK2` on your PC (not on the robot). Follow the instructions in the [Livox-SDK2/README.md](https://github.com/Livox-SDK/Livox-SDK2/blob/master/README.md)
3. Install `livox_ros_driver2` on your PC. Follow the instructions in the [livox_ros_driver2/README.md](https://github.com/Livox-SDK/livox_ros_driver2) and build it in a **separate** workspace (e.g. `~/livox_ws`). Please note that we use ROS2 Humble. 
4. Modify `livox_ros_driver2/config/MID360_config.json`, mainly the network settings to match your setup. Below is an example configuration:
    ```json
    "MID360": {
        "lidar_net_info" : {
        "cmd_data_port": 56100,
        "push_msg_port": 56200,
        "point_data_port": 56300,
        "imu_data_port": 56400,
        "log_data_port": 56500
        },
        "host_net_info" : {
        "cmd_data_ip" : "192.168.123.200",
        "cmd_data_port": 56101,
        "push_msg_ip": "192.168.123.200",
        "push_msg_port": 56201,
        "point_data_ip": "192.168.123.200",
        "point_data_port": 56301,
        "imu_data_ip" : "192.168.123.200",
        "imu_data_port": 56401,
        "log_data_ip" : "",
        "log_data_port": 56501
        }
    },
    "lidar_configs" : [
        {
        "ip" : "192.168.123.20",
        "pcl_data_type" : 1,
        "pattern_mode" : 0,
        "extrinsic_parameter" : {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "x": 0,
            "y": 0,
            "z": 0
        }
        }
    ]
    ```
5. Install `FAST-LIO 2` on your PC. Follow the instructions in the [FAST-LIO 2/README.md](https://github.com/hku-mars/FAST_LIO/tree/ROS2) and build it in a **separate** workspace (e.g. `~/fast_lio_ws`). Make sure you use the ROS2 version. 
   > Please make sure that both `livox_ros_driver2` and `FAST-LIO 2` are built in **separate** workspaces to avoid conflicts.
   > Before building `FAST-LIO 2`, make sure to source the `livox_ws/install/setup.bash` file in your terminal.

#### Docker option

If you prefer to use Docker on the PC, a dedicated mapping image is provided from the `legged_ros2` repository root:

```bash
docker/build_mapping.sh
docker/run_mapping.sh
```

`docker/run_mapping.sh` starts a mapping-specific container, mounts the local `legged_ros2` repository into `/root/legged_ws/src/legged_ros2`, initializes the `legged_ws` workspace, and then opens an interactive shell in the container.

The mapping image already includes:

- `Livox-SDK2`
- `livox_ros_driver2` in `/root/livox_ws`
- `FAST-LIO 2` in `/root/fast_lio_ws`

The mapping Docker workflow uses fixed config paths:

- Livox config: `legged_mapping/config/MID360_config.json` on the host, mounted to `/root/livox_ws/src/livox_ros_driver2/config/MID360_config.json` in the container
- FAST-LIO config: built-in `mid360.yaml` inside the image

After mounting `legged_mapping/config/MID360_config.json`, `docker/run_mapping.sh` will automatically re-run `./build.sh humble` inside `/root/livox_ws/src/livox_ros_driver2` to make sure the Livox driver uses the mounted config.

Open additional terminals in the same running container with:

```bash
docker/enter_mapping.sh
```

Before launching mapping nodes in the container, set `NET_IF` to the local network interface connected to the robot/MID360 network:

```bash
export NET_IF=<your-local-network-interface>
```

For example, if your PC connects to the robot through `enp3s0`:

```bash
export NET_IF=enp3s0
```

You can check the available host network interfaces with `ip addr`. Make sure to export `NET_IF` in the shell before running the mapping launch commands or the helper script.

#### Launch Mapping

You need 4 terminals to launch the mapping process:

1. **Terminal 1**: Source and launch the `livox_ros_driver2` node to get data from MID360:
    ```bash
    export NET_IF=<your-local-network-interface>
    source /root/legged_ws/setup.sh
    export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
    source /root/livox_ws/install/setup.bash
    ros2 launch livox_ros_driver2 msg_MID360_launch.py
    ```
2. **Terminal 2**: Source and launch the `FAST-LIO 2` node to perform SLAM:
    ```bash
    export NET_IF=<your-local-network-interface>
    source /root/legged_ws/setup.sh
    export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
    source /root/livox_ws/install/setup.bash
    source /root/fast_lio_ws/install/setup.bash
    ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
    ```
3. **Terminal 3**: Source and launch the configurable static TF node to bridge the external FAST-LIO frames to the robot base:
    ```bash
    export NET_IF=<your-local-network-interface>
    source /root/legged_ws/setup.sh
    ros2 launch go2_description lidar_static_tf.launch.py
    ```
4. **Terminal 4**: (Optional) Broadcast the robot's TF tree and visualize robot in RViz2:
    ```bash
    export NET_IF=<your-local-network-interface>
    source /root/legged_ws/setup.sh
    ros2 launch go2_description bringup_broadcasters.launch.py 
    ```

If you use the helper script in `scripts/run_mapping_terminals.sh`, export `NET_IF` once before running it:

```bash
export NET_IF=<your-local-network-interface>
bash /root/legged_ws/src/legged_ros2/scripts/run_mapping_terminals.sh
```

The helper script requires `xterm` in the container. It keeps the parent shell waiting after opening the mapping windows; press `Ctrl-C` in that parent shell to close all mapping `xterm` windows.

#### Static TF configuration

`lidar_static_tf_node` publishes two static TF edges that surround the dynamic transform produced by your LIO stack:

- Semantic target: `odom -> tracking_origin -> tracking_body -> base`
- FAST-LIO default compatibility: `odom -> camera_init -> body -> base`

The default parameters live in `legged_mapping/config/lidar_static_tf.yaml` and expose:

- `rotation_order`
- `angle_unit`
- `odom_to_tracking_origin.parent_frame`
- `odom_to_tracking_origin.child_frame`
- `odom_to_tracking_origin.translation_xyz`
- `odom_to_tracking_origin.rotation_angles`
- `tracking_body_to_base.parent_frame`
- `tracking_body_to_base.child_frame`
- `tracking_body_to_base.translation_xyz`
- `tracking_body_to_base.rotation_angles`

`translation_xyz` is in meters.

`rotation_angles` is interpreted using the shared node-level `rotation_order` and `angle_unit` parameters. The three values are no longer implicitly fixed to `roll, pitch, yaw`; they are interpreted according to the selected intrinsic or extrinsic Euler order.

The default file is set up for vanilla FAST-LIO frame names:

- `odom -> camera_init`
- `body -> base`

The current default example uses:

- `rotation_order: extrinsic_xyz`
- `angle_unit: deg`

If your external LIO stack uses different frame names, only update the YAML file. The node code does not need to change.

You can also launch the node with a custom parameter file:

```bash
ros2 launch legged_mapping lidar_static_tf.launch.py \
  params_file:=/absolute/path/to/your_tf.yaml
```

#### Offline transform inversion helper

For manual calibration work, `legged_mapping/scripts/invert_homogeneous_transform.py` provides a standalone CLI helper that does not depend on ROS runtime.

It accepts translation, Euler angles, an explicit intrinsic/extrinsic order string, and `--angle-unit {rad,deg}`. It prints the original homogeneous transform, the inverse transform, the inverse translation, and the inverse Euler angles. The script uses `numpy` internally.

The script and `lidar_static_tf_node` use the same rotation-order enum set and the same angle-unit convention, so you can compute values offline and copy them directly into `lidar_static_tf.yaml`.

Example:

```bash
python3 src/legged_ros2/legged_mapping/scripts/invert_homogeneous_transform.py \
  --translation 1 2 3 \
  --rotation 10 20 30 \
  --order extrinsic_zyx \
  --angle-unit deg
```



### Unitree G1 with MID360

Will be added soon.

### Unitree Go2 with XT16

Will be added soon.
