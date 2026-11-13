# AGENTS.md

Notes for AI coding agents working on this repo (mir_robot: ROS 1 Noetic drivers
and configuration for MiR mobile robots).

## Relation to other repos

This repo contains a ROS driver and all configuration and launch files necessary
to run the MiR robots in ROS both on the real robot and in Gazebo.

At DFKI (where this repo is developed), this MiR robot base is combined with other hardware
(such as a UR5 arm, a gripper, 3D camera etc.) to form the Mobipick robot
(https://github.com/DFKI-NI/mobipick). On top of that, the
[mobipick_labs](https://github.com/DFKI-NI/mobipick_labs) demo environment adds a demo
scenario using the Mobipick robot.

The `mir_robot` repo is on the lowest of these layers and must remain agnostic of the
DFKI-specific upper layers, because it is widely used in other institutions. No references
or dependencies on any of the Mobipick repos must be leaked to the `mir_robot` repo.

Only exception: The `mir_driver/test/` directory includes a bag file that was recorded
on the Mobipick robot and contains the topics "/mobipick/b_scan" and "/mobipick/f_scan".
References to these topic names within this test folder are allowed.

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

- `catkin run_tests` runs the static `roslaunch_add_file_check` XML checks
  registered in the CMakeLists of mir_driver, mir_description, mir_gazebo and
  mir_navigation, plus the `path_follower_test` gtest/rostest in
  mir_dwb_critics. There are no Gazebo tests.
- `mir_dwb_critics` also ships a debug tool `replay_path_follower`
  (test/replay_path_follower.cpp) that replays recorded poses against a
  recorded plan; it is built EXCLUDE_FROM_ALL and never installed, so build
  it explicitly when needed.
- `mir_dwb_critics` forces `CMAKE_BUILD_TYPE=Release` when none is set (the
  critics score ~1000 trajectories per control cycle; an unoptimized build
  misses the controller rate), so don't be surprised by that cache entry.
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

## Package map and ground rules

- `mir_dwb_critics`: only C++ code (pluginlib critics for dwb_local_planner,
  plus unit tests and Python helper scripts). Notable critics: `PathFollower`
  (smooth velocity profile: cruise, brake into stop points, turn in place)
  and `BrakingObstacleFootprint` (footprint check limited to the braking
  distance).
- `mir_driver`: only Python package (`setup.py` + `catkin_python_setup()`).
- `mir_msgs`, `mir_actions`, `sdc21x0`: msg/action/srv definitions, generated
  at build time; nothing generated is committed. These packages are vendored
  (and adapted) from the original MiR software on the robot and must not be
  changed, except when updating to a new software version on the robot. In that
  case, the package from the robot is extracted, the message definitions are
  copied over and (if necessary) merged with the existing ones. Adding fields
  to messages published by the MiR is safe (downward compatible) because they
  will be filled with defaults on older versions, but removing fields breaks
  compatibility with older versions. For messages/action fields subscribed to
  by the robot, the opposite is true: Removing fields is safe, adding fields
  breaks downward comparibility. This has to be carefully considered.
- `mir_description`, `mir_gazebo`, `mir_navigation`: URDF/meshes, launch and
  config. These packages are config-only and must stay that way (no compiled
  code; only exception: some Python helper scripts).
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
- The `SBPLLatticePlanner` comes from `navigation_experimental`, which the
  workspace clones and builds from source (Noetic is EOL so there are no binary
  releases). This is necessary to support the exact goal blending feature added
  in ros-planning/navigation_experimental#64 and enabled by the
  `goal_blend_distance` param in `sbpl_global_params.yaml`).
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
