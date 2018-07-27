mir_driver
==========

This repo contains a ROS driver and ROS configuration files (URDF description,
Gazebo launch files, move_base config, bringup launch files, message and action
descriptions) for the [MiR 100 robot](http://www.mobile-industrial-robots.com/en/products/mir100/).

Package overview
----------------

* `mir_actions`: Action definitions for the MiR robot
* `mir_description`: URDF description of the MiR robot
* `mir_driver`: A reverse ROS bridge for the MiR robot
* `mir_gazebo`: Simulation specific launch and configuration files for the MiR robot
* `mir_msgs`: Message definitions for the MiR robot
* `mir_navigation`: move_base launch and configuration files


Installation
------------

### From binaries

```bash
sudo apt install ros-$ROS_DISTRO-mir-robot
```

### From source

```bash
# install sbpl library from source
cd $(mktemp -d)
git clone -b master https://github.com/sbpl/sbpl.git
cd sbpl
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
sudo make install

# create a catkin workspace and clone all required ROS packages
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src/
git clone -b kinetic https://github.com/dfki-ric/mir_robot.git
git clone -b indigo-devel https://github.com/ricardo-samaniego/sbpl_lattice_planner.git

# use rosdep to install all dependencies (including ROS itself)
apt-get update -qq
apt-get install -qq -y python-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths ./ -i -y --rosdistro kinetic --skip-keys=sbpl

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

Gazebo demo
-----------

```bash
roslaunch mir_gazebo mir_maze_world.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI
roslaunch mir_gazebo fake_localization.launch delta_x:=-10.0 delta_y:=-10.0
roslaunch mir_navigation start_planner.launch \
    map_file:=$(rospack find mir_gazebo)/maps/maze.yaml \
    virtual_walls_map_file:=$(rospack find mir_gazebo)/maps/maze_virtual_walls.yaml \
    local_planner:=eband
rviz -d $(rospack find mir_navigation)/rviz/navigation.rviz
```

Now, you can use the "2D Nav Goal" tool in RViz to set a navigation goal for move_base.

[![MiR100 robot: navigation in Gazebo (2x)](https://i.vimeocdn.com/video/712859121.jpg?mw=640)](https://vimeo.com/279628049)
(Click image to see video)


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

| Indigo |
|--------|
| [![Build Status](https://travis-ci.org/dfki-ric/mir_robot.svg?branch=indigo)](https://travis-ci.org/dfki-ric/mir_robot) |


ROS Buildfarm
-------------

|           | Indigo source deb | Indigo binary deb | Kinetic source deb | Kinetic binary deb |
|-----------|-------------------|-------------------|--------------------|--------------------|
| [mir_actions](http://wiki.ros.org/mir_actions) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Isrc_uT__mir_actions__ubuntu_trusty__source)](http://build.ros.org/job/Isrc_uT__mir_actions__ubuntu_trusty__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__mir_actions__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__mir_actions__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__mir_actions__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__mir_actions__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__mir_actions__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__mir_actions__ubuntu_xenial_amd64__binary/) |
| [mir_description](http://wiki.ros.org/mir_description) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Isrc_uT__mir_description__ubuntu_trusty__source)](http://build.ros.org/job/Isrc_uT__mir_description__ubuntu_trusty__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__mir_description__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__mir_description__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__mir_description__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__mir_description__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__mir_description__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__mir_description__ubuntu_xenial_amd64__binary/) |
| [mir_driver](http://wiki.ros.org/mir_driver) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Isrc_uT__mir_driver__ubuntu_trusty__source)](http://build.ros.org/job/Isrc_uT__mir_driver__ubuntu_trusty__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__mir_driver__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__mir_driver__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__mir_driver__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__mir_driver__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__mir_driver__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__mir_driver__ubuntu_xenial_amd64__binary/) |
| [mir_gazebo](http://wiki.ros.org/mir_gazebo) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Isrc_uT__mir_gazebo__ubuntu_trusty__source)](http://build.ros.org/job/Isrc_uT__mir_gazebo__ubuntu_trusty__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__mir_gazebo__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__mir_gazebo__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__mir_gazebo__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__mir_gazebo__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__mir_gazebo__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__mir_gazebo__ubuntu_xenial_amd64__binary/) |
| [mir_msgs](http://wiki.ros.org/mir_msgs) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Isrc_uT__mir_msgs__ubuntu_trusty__source)](http://build.ros.org/job/Isrc_uT__mir_msgs__ubuntu_trusty__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__mir_msgs__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__mir_msgs__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__mir_msgs__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__mir_msgs__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__mir_msgs__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__mir_msgs__ubuntu_xenial_amd64__binary/) |
| [mir_navigation](http://wiki.ros.org/mir_navigation) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Isrc_uT__mir_navigation__ubuntu_trusty__source)](http://build.ros.org/job/Isrc_uT__mir_navigation__ubuntu_trusty__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__mir_navigation__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__mir_navigation__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__mir_navigation__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__mir_navigation__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__mir_navigation__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__mir_navigation__ubuntu_xenial_amd64__binary/) |
| [mir_robot](http://wiki.ros.org/mir_robot) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Isrc_uT__mir_robot__ubuntu_trusty__source)](http://build.ros.org/job/Isrc_uT__mir_robot__ubuntu_trusty__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ibin_uT64__mir_robot__ubuntu_trusty_amd64__binary)](http://build.ros.org/job/Ibin_uT64__mir_robot__ubuntu_trusty_amd64__binary/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Ksrc_uX__mir_robot__ubuntu_xenial__source)](http://build.ros.org/job/Ksrc_uX__mir_robot__ubuntu_xenial__source/) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__mir_robot__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__mir_robot__ubuntu_xenial_amd64__binary/) |

|           | Indigo devel | Indigo doc | Kinetic devel | Kinetic doc |
|-----------|--------------|------------|---------------|-------------|
| [mir_robot](http://wiki.ros.org/mir_robot) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Idev__mir_robot__ubuntu_trusty_amd64)](http://build.ros.org/job/Idev__mir_robot__ubuntu_trusty_amd64) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Idoc__mir_robot__ubuntu_trusty_amd64)](http://build.ros.org/job/Idoc__mir_robot__ubuntu_trusty_amd64) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdev__mir_robot__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdev__mir_robot__ubuntu_xenial_amd64) | [![Build Status](http://build.ros.org/buildStatus/icon?job=Kdoc__mir_robot__ubuntu_xenial_amd64)](http://build.ros.org/job/Kdoc__mir_robot__ubuntu_xenial_amd64) |
