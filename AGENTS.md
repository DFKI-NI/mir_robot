# AGENTS.md

Notes for AI coding agents working on this repo (mir_robot: ROS 1 Noetic drivers
and configuration for MiR mobile robots).

## Workspace layout

- ROS 1 (Noetic) catkin repo. This checkout is a package inside a larger
  catkin_tools workspace; it is NOT a standalone
  workspace. Run `catkin build`/`catkin run_tests` from the parent workspace
  root, and source the `setup.bash` before `roslaunch`
  or `rosrun`.
- README's install section shows `catkin_make`, but CI
  (`.github/workflows/github-actions.yml`) and `Dockerfile-noetic` use
  `catkin_tools` with a serial build (`catkin config -j 1 -p 1`). Follow CI.
- The primary development branch is `noetic` (README instructs cloning with
  `-b noetic`).

## Build and test

Build/test commands (run from the parent workspace root, noetic sourced):

    catkin config -j 1 -p 1
    catkin build
    catkin build --make-args tests
    catkin run_tests
    catkin_test_results

- The repo has no unit or Gazebo tests. `catkin run_tests` only executes the
  static `roslaunch_add_file_check` XML checks registered in the CMakeLists of
  mir_driver, mir_description, mir_gazebo and mir_navigation - a passing run
  mostly proves the launch files parse.
- `mir_dwb_critics` is the only C++ package and builds with `-Wall -Werror`;
  any compiler warning fails the build.

## Lint and formatting

- CI runs `pre-commit run -a` in the repo checkout; run it before committing.
  Never commit anything that doesn't pass the pre-commit check.
- Python: black + flake8 + pydocstyle, line length 120 (`pyproject.toml` and
  `.flake8` must stay in sync), target py38, no string normalization.
- New `.py` files must carry the license header from `LICENSE`
  (`pre-commit` auto-inserts it; see the exclusion list in
  `.pre-commit-config.yaml`).
- C++ is formatted with `clang-format-10` (exactly version 10).
- `catkin_lint --strict` validates every `package.xml` and `CMakeLists.txt`.

## Package map

- `mir_dwb_critics`: only C++ code (pluginlib critics for dwb_local_planner),
  plus some Python helper scripts.
- `mir_driver`: only Python package (`setup.py` + `catkin_python_setup()`).
- `mir_msgs`, `mir_actions`, `sdc21x0`: msg/action/srv definitions, generated
  at build time; nothing generated is committed.
- `mir_description`, `mir_gazebo`, `mir_navigation`: URDF/meshes, launch and
  config only (no compiled code, only some Python helper scripts).
- `mir_robot`: empty metapackage.

## mir_driver (reverse ROS bridge)

- Talks the rosbridge WebSocket protocol (port 9090) to the robot at
  `mir_hostname` (default `192.168.12.20`) - not the MiR REST API.
- Subscribes lazily and never unsubscribes (the robot refuses to re-subscribe).
- `cmd_vel` is forwarded as `TwistStamped` (MiR >= 2.7); `tf_prefix` is
  prepended to all frames except `map`; `tf_static` is cached and re-published.
- `move_base_simple/goal` is converted to a move_base action goal. If no
  planner runs on the robot, goals are silently dropped - activate "Planner"
  in the MiR web interface.
- Supported MiR software versions: 2.8.3.1 - 2.13.5.x. Do not upgrade robots
  to MiR software 3.0.

## Simulation and navigation quirks

- No URDF is committed: it is generated at runtime from
  `mir_description/urdf/*.xacro`. Default robot is MiR 100; use
  `mir_type:=mir_250` on the gazebo launch files.
- `mir_navigation` exec-depends on `mir_gazebo`: default `map_file` args point
  into `mir_gazebo/maps/`.
- Of the committed `.mprim` files, only `unicycle_highcost_5cm.mprim` is
  script-generated (`mir_navigation/mprim/genmprim_unicycle_highcost_5cm.py`,
  overwrites the file in-tree and needs a display for matplotlib); the rest
  are hand-tuned. The lattice planner in `move_base.xml` uses
  `unicycle_highcost_5cm.mprim`.
- `sdc21x0` is a message-only vendored package (motor controller msgs/srv);
  its `Encoders.msg` name collides with `mir_msgs/Encoders.msg` - check which
  one you are editing.
- Gazebo starts paused: call `rosservice call /gazebo/unpause_physics`.
- Expected noise: Gazebo "No p gain specified for pid." errors and the
  move_base "Got a result when we were already in the DONE state" warning
  (see README Troubleshooting).

## Docker

- `Dockerfile-noetic` is a reproducible CI-style build image (build + tests +
  pre-commit), not a runtime/demo image.
