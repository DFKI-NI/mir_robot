^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mir_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
