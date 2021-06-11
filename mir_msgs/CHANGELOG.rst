^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mir_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.8 (2021-06-11)
------------------
* Merge branch 'melodic-2.8' into melodic
* Update BMSData msg to MiR software 2.8.3.1
* Remove MirStatus
  This message was removed in MiR software 2.0 (Renamed to RobotStatus).
* Update mir_msgs to 2.8.2.2
* Contributors: Felix, Martin Günther

1.0.7 (2021-02-11)
------------------

1.0.6 (2020-06-30)
------------------
* Fix some catkin_lint warnings
* Set cmake_policy CMP0048 to fix warning
* Contributors: Martin Günther

1.0.5 (2020-05-01)
------------------

1.0.4 (2019-05-06)
------------------
* Update mir_msgs and mir_actions to MiR 2.3.1
  The following changes were made to the actual mir_msgs:
  * rename mirMsgs -> mir_msgs
  * rename proximity -> Proximity
  * rename serial -> Serial
  * keep MirStatus msg (was replaced by RobotStatus in MiR software 2.0)
* Contributors: Martin Günther

1.0.3 (2019-03-04)
------------------
* mir_msgs: Compile new msgs + rename mirMsgs -> mir_msgs
* mir_msgs: Add geometry_msgs dependency
  Now that we have an actual msg package dependency, we don't need the std_msgs placeholder any more.
* mir_msgs: Add new messages on kinetic
* Contributors: Martin Günther

1.0.2 (2018-07-30)
------------------

1.0.1 (2018-07-17)
------------------

1.0.0 (2018-07-12)
------------------
* Initial release
* Contributors: Martin Günther
