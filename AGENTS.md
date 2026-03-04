# Repository Guidelines

## Project Structure & Module Organization
- `legged_rl_controller/`: RL controller plugin and supporting C++ headers/sources.
- `legged_ros2_control/`: ROS 2 hardware interface implementations (e.g., Unitree Go2).
- `legged_ros2_controller/`: ROS 2 controllers built on top of ros2_control.
- `legged_robot_description/go2_description/`: URDF/Xacro, meshes, configs, and launch files.
- `third_party/`: vendored dependencies or external assets.

## Build, Test, and Development Commands
Run from the workspace root (e.g., `~/legged_ws`), not the package directory:
- `source /opt/ros/humble/setup.bash`: load ROS 2 environment.
- `colcon build --symlink-install`: build all packages in the workspace.
- `colcon build --packages-select legged_ros2_control`: rebuild a single package.
- `colcon test --packages-select legged_ros2_control`: run package tests/lints.
- `colcon test-result --verbose`: inspect test and lint results.
- `ros2 launch go2_description bringup_rl.launch.py`: launch the RL bringup for Go2.

## Coding Style & Naming Conventions
- C++ is the primary language; headers live in `include/`, sources in `src/`.
- Indentation: 2 spaces for C++ and 4 spaces for Python launch files.
- Naming: `CamelCase` for classes/types, `snake_case` for variables/functions, and
  `snake_case` filenames (e.g., `legged_system_interface.cpp`).
- Follow ROS 2/ament conventions; keep public APIs in headers and register plugins
  through `pluginlib` XML when applicable.

## Testing Guidelines
- Packages enable `ament_lint_auto`/`ament_lint_common` when `BUILD_TESTING` is on.
- There are no dedicated unit tests currently; if you add tests, place them in a
  package-level `test/` directory and document how to run them with `colcon test`.

## Commit & Pull Request Guidelines
- Recent commits use short, imperative subjects (e.g., “fix bug”, “update readme”).
  Keep subjects single-line and focused; include context in the body if needed.
- PRs should describe behavior changes, link relevant issues, and list the `colcon`
  commands used to validate the change.

## Configuration & Simulation Notes
- Sim2Sim/Sim2Real stability depends on joint order, MuJoCo joint `armature`,
  IMU mounting, and matching joint limits; review `README.md` and robot configs
  when updating models or policies.

## Language Requirements
- Always talk to the user in Simplified Chinese. 
- Write code in English
- Language used in documentation should ask the user to choose between English and Simplified Chinese, and then use the chosen language for the specified documentation.
