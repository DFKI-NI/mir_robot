^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mir_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.6 (2020-06-30)
------------------
* Set cmake_policy CMP0048 to fix warning
* Contributors: Martin Günther

1.0.5 (2020-05-01)
------------------
* Add optional prefix parameter to fake_mir_joint_publisher (`#47 <https://github.com/dfki-ric/mir_robot/issues/47>`_)
* tf_remove_child_frames: Don't publish empty TFs
* Add sdc21x0 package, MC/currents topic
* Contributors: Martin Günther, Nils Niemann

1.0.4 (2019-05-06)
------------------
* Remove garbage file
* Contributors: Martin Günther

1.0.3 (2019-03-04)
------------------
* Make disable_map work with MiR software 2.0
  See `#5 <https://github.com/dfki-ric/mir_robot/issues/5>`_.
* mir_driver: Optionally disable the map topic + TF frame (`#6 <https://github.com/dfki-ric/mir_robot/issues/6>`_)
  This is useful when running one's own SLAM / localization nodes.
  Fixes `#5 <https://github.com/dfki-ric/mir_robot/issues/5>`_.
* Split scan_rep117 topic into two separate topics
  This fixes the problem that the back laser scanner was ignored in the
  navigation costmap in Gazebo (probably because in Gazebo, both laser
  scanners have the exact same timestamp).
* Contributors: Martin Günther

1.0.2 (2018-07-30)
------------------

1.0.1 (2018-07-17)
------------------
* mir_driver: Remove leading slashes in TF frames
* mir_driver: Install launch directory
* Contributors: Martin Günther

1.0.0 (2018-07-12)
------------------
* Initial release
* Contributors: Martin Günther
