# Contributing

Issues and pull requests are welcome.

## Setup

- Ubuntu 24.04, ROS 2 Jazzy, MoveIt 2 with MoveIt Task Constructor
- Isaac Sim 5.x/6.x, only for `hardware:=isaac`
- Build the workspace as described in the [README](README.md#quick-start)

## Reporting a bug

Open an issue and include:

- the launch command and its arguments (`side`, `hardware`, `cube_pose_mode`)
- what you expected, and what happened instead
- the relevant console output (the `mtc_node` and `move_group` lines are usually enough)

## Pull requests

1. Fork the repository and create a branch: `git checkout -b feat/short-description`
2. Keep each change focused. A new motion primitive goes in
   `ur5e_rh56dfx_manipulation/include/ur5e_rh56dfx_manipulation/motion_primitives/`, with its stage factory
   next to it.
3. Check that the demo still runs with mock hardware, for both hands:
   ```bash
   colcon build --symlink-install
   ros2 launch ur5e_rh56dfx_manipulation mtc_demo.launch.py side:=left
   ros2 launch ur5e_rh56dfx_manipulation mtc_demo.launch.py side:=right
   ```
4. Use [Conventional Commits](https://www.conventionalcommits.org/) messages, for example
   `feat: add a side-grasp generator` or `fix: stop the thumb at first contact`.
5. Open the pull request against `main` and describe what changed and how you tested it.

## Code style

- C++17, following the [ROS 2 code style](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- Python: PEP 8
- Comments explain why, not what

By contributing, you agree that your contributions are licensed under the [Apache 2.0 License](LICENSE).
