mir_robot
=========

This repo contains a ROS driver and ROS configuration files (URDF description,
Gazebo launch files, move_base config, bringup launch files, message and action
descriptions) for the [MiR robots](http://www.mobile-industrial-robots.com/).
This is a community project created by us ([DFKI](https://www.dfki.de/), the
German Research Center for Artificial Intelligence) to use the MiR Robots with
ROS. We are not affiliated with Mobile Industrial Robots. If you find a bug or
missing feature in this software, please report it on the
[issue tracker](https://github.com/DFKI-NI/mir_robot/issues).

Supported MiR robots and software versions
------------------------------------------

This repo has been confirmed to work with the following robots:

* MiR 100
* MiR 200
* MiR 250
* MiR 500

It probably also works with the MiR1000. If you can test it on one
of those, please let us know if it works.

This repo has been tested with the following MiR software versions:

* 2.8.3.1
* 2.13.4.1
* 2.13.5.3

You can try if it works with other versions, but these are the ones that are
known to work.

:warning: **Do NOT update to the upcoming version 3.0; it is very well possible
that this repo will no longer work with this version!**


Package overview
----------------

* `mir_actions`: Action definitions for the MiR robot
* `mir_description`: URDF description of the MiR robot
* `mir_dwb_critics`: Plugins for the dwb_local_planner used in Gazebo
* `mir_driver`: A reverse ROS bridge for the MiR robot
* `mir_gazebo`: Simulation specific launch and configuration files for the MiR robot
* `mir_msgs`: Message definitions for the MiR robot
* `mir_navigation`: move_base launch and configuration files


Installation
------------

You can chose between binary and source install below. If you don't want to
modify the source, the binary install is preferred (if `mir_robot` binary
packages are available for your ROS distro). The instructions below use the ROS
distro `noetic` as an example; if you use a different distro (e.g.  `melodic`),
replace all occurrences of the string `noetic` by your distro name in the
instructions.

### Preliminaries

If you haven't already installed ROS on your PC, you need to add the ROS apt
repository. This step is necessary for either binary or source install.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
```

### Binary install

For a binary install, it suffices to run this command:

```bash
sudo apt install ros-noetic-mir-robot
```

See the tables at the end of this README for a list of ROS distros for which
binary packages are available.

### Source install

For a source install, run the commands below instead of the command from the
"binary install" section.

```bash
# create a catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/

# clone mir_robot into the catkin workspace
git clone -b noetic https://github.com/DFKI-NI/mir_robot.git

# use rosdep to install all dependencies (including ROS itself)
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep
sudo rosdep init
rosdep update --include-eol-distros
rosdep install --from-paths ./ -i -y --rosdistro noetic

# build all packages in the catkin workspace
source /opt/ros/noetic/setup.bash
catkin_init_workspace
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo
```

In case you encounter problems, please compare the commands above to the build
step in [`.github/workflows/github-actions.yml`](.github/workflows/github-actions.yml); that should always have the most
recent list of commands.

You should add the following line to the end of your `~/.bashrc`, and then
close and reopen all terminals:

```bash
source ~/catkin_ws/devel/setup.bash
```

Gazebo demo (existing map)
--------------------------

https://user-images.githubusercontent.com/320188/145610491-2afeb46c-3729-4106-ab9c-6681b5dd9d2e.mp4

```bash
### gazebo:
roslaunch mir_gazebo mir_maze_world.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI

### localization:
roslaunch mir_navigation amcl.launch initial_pose_x:=10.0 initial_pose_y:=10.0
# or alternatively: roslaunch mir_gazebo fake_localization.launch delta_x:=-10.0 delta_y:=-10.0

# navigation:
roslaunch mir_navigation start_planner.launch \
    map_file:=$(rospack find mir_gazebo)/maps/maze.yaml \
    virtual_walls_map_file:=$(rospack find mir_gazebo)/maps/maze_virtual_walls.yaml
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz
```

Now, you can use the "2D Nav Goal" tool in RViz to set a navigation goal for move_base.


Gazebo demo (mapping)
---------------------

```bash
### gazebo:
roslaunch mir_gazebo mir_maze_world.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI

### mapping:
roslaunch mir_navigation hector_mapping.launch

# navigation:
roslaunch mir_navigation move_base.xml with_virtual_walls:=false
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz
```

Gazebo demo (MiR 250 in warehouse Gazebo world)
-----------------------------------------------

https://user-images.githubusercontent.com/320188/171613044-639f3ab2-fe84-4839-acfc-d0642f8869b3.mp4

This repo contains URDF descriptions for the MiR 100 (default) and the MiR 250.
You can switch to the MiR 250 by adding **`mir_type:=mir_250`** to the gazebo
roslaunch command. You can also select another Gazebo world using the
**`world_name`** argument. For example, the video above was generated using the
following commands:

```bash
cd <your catkin workspace>
git clone -b ros1 https://github.com/aws-robotics/aws-robomaker-small-warehouse-world.git
catkin build

roslaunch mir_gazebo mir_empty_world.launch \
        world_name:=$(rospack find aws_robomaker_small_warehouse_world)/worlds/no_roof_small_warehouse.world \
        mir_type:=mir_250
```

... and then running the remaining commands from the "mapping" section above.


Gazebo demo (multiple robots)
-----------------------------

If you want to spawn multiple robots into Gazebo, you unfortunately have to
hard-code the name of the second robot into the `mir_empty_world.launch` file,
like this:

```diff
diff --git i/mir_gazebo/launch/mir_empty_world.launch w/mir_gazebo/launch/mir_empty_world.launch
index 27b9159..7773fae 100644
--- i/mir_gazebo/launch/mir_empty_world.launch
+++ w/mir_gazebo/launch/mir_empty_world.launch
@@ -17,6 +17,10 @@
       <remap from="$(arg namespace)/mobile_base_controller/cmd_vel" to="$(arg namespace)/cmd_vel" />
       <remap from="$(arg namespace)/mobile_base_controller/odom"    to="$(arg namespace)/odom" />

+      <remap from="mir2/joint_states"                   to="mir2/mir/joint_states" />
+      <remap from="mir2/mobile_base_controller/cmd_vel" to="mir2/cmd_vel" />
+      <remap from="mir2/mobile_base_controller/odom"    to="mir2/odom" />
+
       <include file="$(find gazebo_ros)/launch/empty_world.launch">
         <arg name="world_name" value="$(arg world_name)"/>
         <arg name="paused" value="true" />
```

Then you can run the simulation like this:

```bash
# start Gazebo + first MiR
roslaunch mir_gazebo mir_maze_world.launch tf_prefix:=mir

# first MiR: start localization, navigation + rviz
roslaunch mir_navigation amcl.launch initial_pose_x:=10.0 initial_pose_y:=10.0 tf_prefix:=mir
roslaunch mir_navigation start_planner.launch \
        map_file:=$(rospack find mir_gazebo)/maps/maze.yaml \
        virtual_walls_map_file:=$(rospack find mir_gazebo)/maps/maze_virtual_walls.yaml prefix:=mir/
ROS_NAMESPACE=mir rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz

# spawn second MiR into Gazebo
roslaunch mir_gazebo mir_gazebo_common.launch robot_x:=-2 robot_y:=-2 tf_prefix:=mir2 model_name:=mir2 __ns:=mir2

# second MiR: start localization, navigation + rviz
roslaunch mir_navigation amcl.launch initial_pose_x:=8.0 initial_pose_y:=8.0 tf_prefix:=mir2
roslaunch mir_navigation start_planner.launch \
        map_file:=$(rospack find mir_gazebo)/maps/maze.yaml \
        virtual_walls_map_file:=$(rospack find mir_gazebo)/maps/maze_virtual_walls.yaml prefix:=mir2/
ROS_NAMESPACE=mir2 rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz
```


Running the driver on the real robot
------------------------------------

### Start up the robot

* switch on MiR base


### Connect to the MiR web interface

* connect to MiR_R??? wifi (password "mirex4you"), for example from your Android phone/tablet
* disable other network connections (mobile data / LAN / etc.)
* open mir.com (192.168.12.20) in Chrome (!)
* log in (admin/mir4you)


### Synchronize system time

The internal robot PC's is not synchronized (for example via NTP), so it tends
to get out of sync quickly (about 1 second per day!). This causes TF transforms
timing out etc. and can be seen using `tf_monitor` (the "Max Delay" is about
3.3 seconds, but should be less than 0.1 seconds):

```text
$ rosrun tf tf_monitor
Frames:
Frame: /back_laser_link published by unknown_publisher Average Delay: 3.22686 Max Delay: 3.34766
Frame: /base_footprint published by unknown_publisher Average Delay: 3.34273 Max Delay: 3.38062
Frame: /base_link published by unknown_publisher Average Delay: 3.22751 Max Delay: 3.34844
Frame: /front_laser_link published by unknown_publisher Average Delay: 3.22661 Max Delay: 3.34159
Frame: /imu_link published by unknown_publisher Average Delay: 3.22739 Max Delay: 3.34738
Frame: /odom published by unknown_publisher Average Delay: 3.16493 Max Delay: 3.28667
[...]

All Broadcasters:
Node: unknown_publisher 418.344 Hz, Average Delay: 0.827575 Max Delay: 3.35237
Node: unknown_publisher(static) 465.362 Hz, Average Delay: 0 Max Delay: 0
```

To fix this:

* go to "Service" -> "Configuration" -> "System settings" -> "Time settings" -> "Set device time on robot"

Afterwards, the ROS software on the robot will restart, so you'll have to start `move_base` again (see below).

If you have an external PC on the MiR platform, you can use `chrony` to automatically synchronize system time (see below).


### Start `move_base` on the robot

* go to "Service" -> "Configuration" -> "Launch menu", start "Planner"; this starts `move_base` and `amcl` on the robot


### Teleoperate the robot (optional)

* go to "Manual", press yellow button (LEDs change from yellow to blue); now the robot can be teleoperated


### Relocalize robot (optional)

If the robot's localization is lost:

* go to "Service" -> "Command view" -> "Set start position" and click + drag to current position of robot in the map
* click "Adjust"


### Start the ROS driver

```bash
roslaunch mir_driver mir.launch
```

Advanced
--------

### Installing chrony to synchronize system time automatically

If you have an external PC integrated into your robot that is on the same wired
network as the MiR PC, you can use `chrony` to automatically synchronize the
MiR's system time. Unfortunately, this method is not easy to install.

Let's call the external PC `external-pc`. That PC's clock is our reference
clock. It is synced to an NTP clock whenever the `external-pc` has access to
the internet. To implement this synchronization solution, install `chrony` on
both the `external-pc` and the internal PC of the MiR, and set up the
`external-pc` as the chrony server and the internal MiR PC as the chrony
client. This way, the clocks on these systems always stay in sync without any
manual interaction.

To install things on the internal MiR PC:

* Connect a monitor, keyboard and USB stick with a live Linux system to the
  ports that are exposed on one corner of the MiR.
* Boot into the live USB Linux system.
* `chroot` into the MiR PC:

    1. Mount MiR partition and bind /dev, /run etc.
       You can use fdisk -l to figure out which partition to mount.
       (Here it's `sda3`):

       ```bash
       sudo mkdir -p /media/mir
       sudo mount /dev/sda3 /media/mir

       for dir in /dev /dev/pts /proc /sys /run; do sudo mount --bind $dir /media/mir/@$dir; done
       ```

    2. `chroot` into the MiR PC:

       ```bash
       sudo chroot /media/mir/@/
       ```

* Create user:

   ```bash
   adduser newuser
   usermod -aG sudo newuser
   ```

* Reboot and log into MiR PC via SSH and the newly created user name.
* Install Chrony:

   ```bash
   sudo apt update
   sudo apt install chrony

   # if not installable (to fix broken dependencies):
   sudo apt -f install
   sudo apt install chrony
   ```

* Set up `/etc/chrony/chrony.conf`.
* Make sure all old ntp configs are configured in chrony. For this, add the
  following to your chrony.conf (the old ntp.conf part is commented out):

```bash
# Clients from this subnet have unlimited access, but only if
# cryptographically authenticated.
#restrict 192.168.12.255 mask 255.255.255.0 nomodify notrap nopeer
allow 192.168.12.0/24 nomodify notrap nopeer
```

* Restart chrony service.


Troubleshooting
---------------

### Got a result when we were already in the DONE state

Sometimes the move_base action will print the warning "Got a result when we
were already in the DONE state". This is caused by a race condition between the
`/move_base/result` and `/move_base/status` topics. When a status message with
status `SUCCEEDED` arrives before the corresponding result message, this
warning will be printed. It can be safely ignored.


### Gazebo prints errors: "No p gain specified for pid."

These errors are expected and can be ignored.

Unfortunately, we cannot set the PID gains (to silence the error) due to the
following behavior of Gazebo:

1. When using the `PositionJointInterface`, you *must* set the PID values for the
   joints using that interface, otherwise you will run into
   [this bug](https://github.com/ros-simulation/gazebo_ros_pkgs/issues/612).
2. When using the `VelocityJointInterface`, if you omit the PID values, Gazebo
   just perfectly follows the commanded velocities. If you specify PID values,
   Gazebo will use a PID controller to approximate following the commanded
   velocities, so you have to tune the PID controllers.

Since we just want Gazebo to follow our commanded velocities, we cannot set the
PID values for joints using the VelocityJointInterface, so the errors get
printed (but can be ignored).


pre-commit Formatting Checks
----------------------------

This repo has a [pre-commit](https://pre-commit.com/) check that runs in CI.
You can use this locally and set it up to run automatically before you commit
something. To install, use pip:

```bash
pip3 install --user pre-commit
```

To run over all the files in the repo manually:

```bash
pre-commit run -a
```

To run pre-commit automatically before committing in the local repo, install the git hooks:

```bash
pre-commit install
```


GitHub Actions - Continuous Integration
---------------------------------------

| Noetic                                                                                                                                                                               |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [![Build Status](https://github.com/DFKI-NI/mir_robot/actions/workflows/github-actions.yml/badge.svg)](https://github.com/DFKI-NI/mir_robot/actions/workflows/github-actions.yml/) |


ROS Buildfarm
-------------

|                                                        | Noetic source deb                                                                                                                                                                     | Noetic binary deb                                                                                                                                                                                     |
|--------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [mir_actions](http://wiki.ros.org/mir_actions)         | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_actions__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_actions__ubuntu_focal__source/)         | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_actions__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_actions__ubuntu_focal_amd64__binary/)         |
| [mir_description](http://wiki.ros.org/mir_description) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_description__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_description__ubuntu_focal__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_description__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_description__ubuntu_focal_amd64__binary/) |
| [mir_driver](http://wiki.ros.org/mir_driver)           | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_driver__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_driver__ubuntu_focal__source/)           | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_driver__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_driver__ubuntu_focal_amd64__binary/)           |
| [mir_dwb_critics](http://wiki.ros.org/mir_dwb_critics) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_dwb_critics__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_dwb_critics__ubuntu_focal__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_dwb_critics__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_dwb_critics__ubuntu_focal_amd64__binary/) |
| [mir_gazebo](http://wiki.ros.org/mir_gazebo)           | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_gazebo__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_gazebo__ubuntu_focal__source/)           | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_gazebo__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_gazebo__ubuntu_focal_amd64__binary/)           |
| [mir_msgs](http://wiki.ros.org/mir_msgs)               | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_msgs__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_msgs__ubuntu_focal__source/)               | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_msgs__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_msgs__ubuntu_focal_amd64__binary/)               |
| [mir_navigation](http://wiki.ros.org/mir_navigation)   | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_navigation__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_navigation__ubuntu_focal__source/)   | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_navigation__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_navigation__ubuntu_focal_amd64__binary/)   |
| [mir_robot](http://wiki.ros.org/mir_robot)             | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_robot__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_robot__ubuntu_focal__source/)             | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_robot__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_robot__ubuntu_focal_amd64__binary/)             |
| [sdc21x0](http://wiki.ros.org/sdc21x0)                 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__sdc21x0__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__sdc21x0__ubuntu_focal__source/)                 | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__sdc21x0__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__sdc21x0__ubuntu_focal_amd64__binary/)                 |

|                                            | Noetic devel                                                                                                                                                   | Noetic doc                                                                                                                                                     |
|--------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [mir_robot](http://wiki.ros.org/mir_robot) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ndev__mir_robot__ubuntu_focal_amd64)](http://build.ros.org/job/Ndev__mir_robot__ubuntu_focal_amd64) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ndoc__mir_robot__ubuntu_focal_amd64)](http://build.ros.org/job/Ndoc__mir_robot__ubuntu_focal_amd64) |
