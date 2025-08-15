# Unitree Mujoco

Most code here are from [Unitree's official repository](https://github.com/unitreerobotics/unitree_mujoco). Please refer to their documentation for more details.

## Modifications

- Integrate `unitree_mujoco` into ROS2. 
- Add crc32 check in `UnitreeSdk2Bridge` when publishing low states. 
- Add G1 27-dof (29-dof with waist locked) in `unitree_robots`.
- Modify `CMakeLists.txt` in simulate. 
- Some new stl file in `unitree_robots/g1/meshes`.
- Add `mj_ray` to get the surrounding terrain height. 
- Modify `<default>` in robot model xml files to set `<geom>` group as `1`.

