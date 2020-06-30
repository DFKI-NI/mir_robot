mir_driver
==========

This repo contains a ROS driver and ROS configuration files (URDF description,
Gazebo launch files, move_base config, bringup launch files, message and action
descriptions) for the [MiR 100 robot](http://www.mobile-industrial-robots.com/en/products/mir100/).
This is a community project created by us ([DFKI](https://www.dfki.de/), the
German Research Center for Artificial Intelligence) to use the MiR Robots with
ROS. We are not affiliated with Mobile Industrial Robots. If you find a bug or
missing feature in this software, please report it on the
[issue tracker](https://github.com/dfki-ric/mir_robot/issues).

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
distro `kinetic` as an example; if you use a different distro (e.g.  `indigo`),
replace all occurrences of the string `kinetic` by your distro name in the
instructions.

### Preliminaries

If you haven't already installed ROS on your PC, you need to add the ROS apt
repository. This step is necessary for either binary or source install.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq
```

### Binary install

For a binary install, it suffices to run this command:

```bash
sudo apt install ros-kinetic-mir-robot
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
git clone -b kinetic https://github.com/dfki-ric/mir_robot.git

# use rosdep to install all dependencies (including ROS itself)
sudo apt-get update -qq
sudo apt-get install -qq -y python-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths ./ -i -y --rosdistro kinetic

# build all packages in the catkin workspace
source /opt/ros/kinetic/setup.bash
catkin_init_workspace
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebugInfo
```

In case you encounter problems, please compare the commands above to the build
step in [`.travis.yml`](.travis.yml); that should always have the most
recent list of commands.

You should add the following line to the end of your `~/.bashrc`, and then
close and reopen all terminals:

```bash
source ~/catkin_ws/devel/setup.bash
```

Gazebo demo (existing map)
--------------------------

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

[![MiR100 robot: navigation in Gazebo (4x)](https://i.vimeocdn.com/video/859959481.jpg?mw=640)](https://vimeo.com/394184430)

(Click image to see video)


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

```
$ rosrun tf tf_monitor
Frames:
Frame: /back_laser_link published by unknown_publisher Average Delay: 3.22686 Max Delay: 3.34766
Frame: /base_footprint published by unknown_publisher Average Delay: 3.34273 Max Delay: 3.38062
Frame: /base_link published by unknown_publisher Average Delay: 3.22751 Max Delay: 3.34844
Frame: /front_laser_link published by unknown_publisher Average Delay: 3.22661 Max Delay: 3.34159
Frame: /imu_link published by unknown_publisher Average Delay: 3.22739 Max Delay: 3.34738
Frame: /odom_comb published by unknown_publisher Average Delay: 3.16493 Max Delay: 3.28667
[...]

All Broadcasters:
Node: unknown_publisher 418.344 Hz, Average Delay: 0.827575 Max Delay: 3.35237
Node: unknown_publisher(static) 465.362 Hz, Average Delay: 0 Max Delay: 0
```

To fix this:

* go to "Service" -> "Configuration" -> "System settings" -> "Time settings" -> "Set device time on robot"

Afterwards, the ROS software on the robot will restart, so you'll have to start `move_base` again (see below).


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


Travis - Continuous Integration
-------------------------------

| Kinetic | Melodic | Noetic |
|---------|---------|--------|
| [![Build Status](https://travis-ci.org/dfki-ric/mir_robot.svg?branch=kinetic)](https://travis-ci.org/dfki-ric/mir_robot) | [![Build Status](https://travis-ci.org/dfki-ric/mir_robot.svg?branch=melodic)](https://travis-ci.org/dfki-ric/mir_robot) | [![Build Status](https://travis-ci.org/dfki-ric/mir_robot.svg?branch=noetic)](https://travis-ci.org/dfki-ric/mir_robot) |



ROS Buildfarm
-------------

|           | Kinetic source deb | Kinetic binary deb | Melodic source deb | Melodic binary deb | Noetic source deb | Noetic binary deb |
|-----------|--------------------|--------------------|--------------------|--------------------|-------------------|-------------------|
| [mir_actions](http://wiki.ros.org/mir_actions) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__mir_actions__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__mir_actions__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__mir_actions__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__mir_actions__ubuntu_xenial_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__mir_actions__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__mir_actions__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__mir_actions__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__mir_actions__ubuntu_bionic_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_actions__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_actions__ubuntu_focal__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_actions__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_actions__ubuntu_focal_amd64__binary/) |
| [mir_description](http://wiki.ros.org/mir_description) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__mir_description__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__mir_description__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__mir_description__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__mir_description__ubuntu_xenial_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__mir_description__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__mir_description__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__mir_description__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__mir_description__ubuntu_bionic_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_description__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_description__ubuntu_focal__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_description__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_description__ubuntu_focal_amd64__binary/) |
| [mir_driver](http://wiki.ros.org/mir_driver) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__mir_driver__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__mir_driver__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__mir_driver__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__mir_driver__ubuntu_xenial_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__mir_driver__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__mir_driver__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__mir_driver__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__mir_driver__ubuntu_bionic_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_driver__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_driver__ubuntu_focal__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_driver__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_driver__ubuntu_focal_amd64__binary/) |
| [mir_dwb_critics](http://wiki.ros.org/mir_dwb_critics) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__mir_dwb_critics__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__mir_dwb_critics__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__mir_dwb_critics__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__mir_dwb_critics__ubuntu_xenial_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__mir_dwb_critics__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__mir_dwb_critics__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__mir_dwb_critics__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__mir_dwb_critics__ubuntu_bionic_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_dwb_critics__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_dwb_critics__ubuntu_focal__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_dwb_critics__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_dwb_critics__ubuntu_focal_amd64__binary/) |
| [mir_gazebo](http://wiki.ros.org/mir_gazebo) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__mir_gazebo__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__mir_gazebo__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__mir_gazebo__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__mir_gazebo__ubuntu_xenial_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__mir_gazebo__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__mir_gazebo__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__mir_gazebo__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__mir_gazebo__ubuntu_bionic_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_gazebo__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_gazebo__ubuntu_focal__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_gazebo__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_gazebo__ubuntu_focal_amd64__binary/) |
| [mir_msgs](http://wiki.ros.org/mir_msgs) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__mir_msgs__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__mir_msgs__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__mir_msgs__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__mir_msgs__ubuntu_xenial_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__mir_msgs__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__mir_msgs__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__mir_msgs__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__mir_msgs__ubuntu_bionic_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_msgs__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_msgs__ubuntu_focal__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_msgs__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_msgs__ubuntu_focal_amd64__binary/) |
| [mir_navigation](http://wiki.ros.org/mir_navigation) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__mir_navigation__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__mir_navigation__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__mir_navigation__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__mir_navigation__ubuntu_xenial_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__mir_navigation__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__mir_navigation__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__mir_navigation__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__mir_navigation__ubuntu_bionic_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_navigation__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_navigation__ubuntu_focal__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_navigation__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_navigation__ubuntu_focal_amd64__binary/) |
| [mir_robot](http://wiki.ros.org/mir_robot) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__mir_robot__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__mir_robot__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__mir_robot__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__mir_robot__ubuntu_xenial_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__mir_robot__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__mir_robot__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__mir_robot__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__mir_robot__ubuntu_bionic_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__mir_robot__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__mir_robot__ubuntu_focal__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__mir_robot__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__mir_robot__ubuntu_focal_amd64__binary/) |
| [sdc21x0](http://wiki.ros.org/sdc21x0) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__sdc21x0__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__sdc21x0__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__sdc21x0__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__sdc21x0__ubuntu_xenial_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Msrc_uB__sdc21x0__ubuntu_bionic__source)](http://build.ros.org/job/Msrc_uB__sdc21x0__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mbin_uB64__sdc21x0__ubuntu_bionic_amd64__binary)](http://build.ros.org/job/Mbin_uB64__sdc21x0__ubuntu_bionic_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__sdc21x0__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__sdc21x0__ubuntu_focal__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__sdc21x0__ubuntu_focal_amd64__binary)](http://build.ros.org/job/Nbin_uF64__sdc21x0__ubuntu_focal_amd64__binary/) |

|           | Kinetic devel | Kinetic doc | Melodic devel | Melodic doc | Noetic devel | Noetic doc |
|-----------|---------------|-------------|---------------|-------------|--------------|------------|
| [mir_robot](http://wiki.ros.org/mir_robot) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__mir_robot__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdev__mir_robot__ubuntu_xenial_amd64) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdoc__mir_robot__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdoc__mir_robot__ubuntu_xenial_amd64) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdev__mir_robot__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__mir_robot__ubuntu_bionic_amd64) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Mdoc__mir_robot__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdoc__mir_robot__ubuntu_bionic_amd64) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ndev__mir_robot__ubuntu_focal_amd64)](http://build.ros.org/job/Ndev__mir_robot__ubuntu_focal_amd64) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ndoc__mir_robot__ubuntu_focal_amd64)](http://build.ros.org/job/Ndoc__mir_robot__ubuntu_focal_amd64) |
