^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mir_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2019-03-04)
------------------
* fix frame_id for melodic (`#18 <https://github.com/dfki-ric/mir_robot/issues/18>`_)
* Tune dwb parameters
* PathProgressCritic: Add heading score
* Use dwb_local_planner in move_base config
* Move footprint param to move_base root namespace
  This allows other move_base plugins, such as dwb_local_planner, to
  access this parameter.
* Add hector_mapping
* amcl.launch: Change default, remap service
  This is required if amcl.launch is started within a namespace.
* teb_local_planner: Fix odom topic name
* Merge pull request `#16 <https://github.com/dfki-ric/mir_robot/issues/16>`_ from niniemann/add-prefix-argument-to-configs
  Add prefix argument to configs
* adds $(arg prefix) to a lot of configs
  This is an important step to be able to re-parameterize move base,
  the diffdrive controller, ekf, amcl and the costmaps for adding a
  tf prefix to the robots links
* mir_navigation: Adjust helper node topics
* Add amcl launchfile (`#11 <https://github.com/dfki-ric/mir_robot/issues/11>`_)
  * added amcl.launch
  * changed amcl params to default mir amcl parameters
* Merge pull request `#13 <https://github.com/dfki-ric/mir_robot/issues/13>`_ from niniemann/fix-virtual-walls
  The previous configuration of the local costmap didn't work for me -- obstacles seen in the laser scans were not added, or were overridden by the virtual\_walls\_map layer. Reordering the layers and loading the virtual walls before the obstacles fixes this for me.
  Also, I added a `with_virtual_walls` parameter to `start_maps.launch` and `start_planner.launch`.
* added with_virtual_walls parameter to start_maps and start_planner
* reorder local costmap plugins
* Revert "mir_navigation: Disable virtual walls if no map file set"
  This reverts commit 0cfda301b2bb1e8b3458e698efd24a7901e5d132.
  The reason is that the `eval` keyword was introduced in kinetic, so it
  doesn't work in indigo.
* mir_navigation: Update rviz config
* mir_navigation: Disable virtual walls if no map file set
* mir_navigation: Rename virtual_walls args + files
* mir_navigation: Remove parameter first_map_only
  This parameter must be set to false (the default) when running SLAM
  (otherwise the map updates won't be received), and when running a static
  map_server it doesn't matter; even then, it should be false to allow
  restarting the map_server with a different map. Therefore this commit
  removes it altogether and leaves it at the default of "false".
* split parameter files between mapping/planning (`#10 <https://github.com/dfki-ric/mir_robot/issues/10>`_)
  The differences are simple: When mapping, first_map_only must be
  set to false, and the virtual walls plugin must not be loaded
  (else move_base will wait for a topic that is not going to be
  published).
* Document move_base params, add max_planning_retries
  Setting max_planning_retries to 10 makes the planner fail faster if the
  planning problem is infeasible. By default, there's an infinite number
  of retries, so we had to wait until the planner_patience ran out (5 s).
* Update rviz config
  Make topics relative, so that ROS_NAMESPACE=... works.
* Switch to binary sbpl_lattice_planner dependency
  ... instead of compiling from source.
* Split scan_rep117 topic into two separate topics
  This fixes the problem that the back laser scanner was ignored in the
  navigation costmap in Gazebo (probably because in Gazebo, both laser
  scanners have the exact same timestamp).
* mir_navigation: Add clear_params to move_base launch
* mir_navigation: marking + clearing were switched
  Other than misleading names, this had no effect.
* Contributors: Martin Günther, Nils Niemann, Noël Martignoni

1.0.2 (2018-07-30)
------------------

1.0.1 (2018-07-17)
------------------

1.0.0 (2018-07-12)
------------------
* Initial release
* Contributors: Martin Günther
