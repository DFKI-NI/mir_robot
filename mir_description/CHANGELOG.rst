^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mir_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.6 (2020-06-30)
------------------
* Update to non-deprecated robot_state_publisher node
* Set cmake_policy CMP0048 to fix warning
* Contributors: Martin Günther

1.0.5 (2020-05-01)
------------------
* Switch from Gazebo GPU laser to normal laser plugin
  The GPU laser plugin has caused multiple people problems before, because
  it is not compatible with all GPUS: `#1 <https://github.com/dfki-ric/mir_robot/issues/1>`_ 
  `#32 <https://github.com/dfki-ric/mir_robot/issues/32>`_ 
  `#46 <https://github.com/dfki-ric/mir_robot/issues/46>`_ 
  `#52 <https://github.com/dfki-ric/mir_robot/issues/52>`_
  The normal laser plugin directly uses the physics engine, so it doesn't
  depend on any specific GPU. Also, it doesn't slow down the simulation
  noticeably (maybe 1-2%).
* Contributors: Martin Günther

1.0.4 (2019-05-06)
------------------
* Add legacyModeNS param to gazebo_ros_control plugin
  This enables the new behavior of the plugin (pid_gains parameter are now
  in the proper namespace).
* re-added gazebo friction parameters for the wheels (`#19 <https://github.com/dfki-ric/mir_robot/issues/19>`_)
* Contributors: Martin Günther, niniemann

1.0.3 (2019-03-04)
------------------
* Merge pull request `#16 <https://github.com/dfki-ric/mir_robot/issues/16>`_ from niniemann/add-prefix-argument-to-configs
  Add prefix argument to configs
* removed prefix from plugin frameName in sick urdf
  The gazebo plugins automatically use tf_prefix, even if none is set
  (in that case it defaults to the robot namespace). That's why we can
  remove the prefix from the plugins configuration, assuming that the
  robot namespace will be equal to the prefix.
* adds $(arg prefix) to a lot of configs
  This is an important step to be able to re-parameterize move base,
  the diffdrive controller, ekf, amcl and the costmaps for adding a
  tf prefix to the robots links
* workaround eval in xacro for indigo support
* adds tf_prefix argument to imu.gazebo.urdf.xacro
* Add TFs for ultrasound sensors
* Contributors: Martin Günther, Nils Niemann

1.0.2 (2018-07-30)
------------------

1.0.1 (2018-07-17)
------------------
* gazebo: Remove leading slashes in TF frames
  TF2 doesn't like it (e.g., robot_localization).
* Contributors: Martin Günther

1.0.0 (2018-07-12)
------------------
* Initial release
* Contributors: Martin Günther
