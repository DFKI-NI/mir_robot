^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mir_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.7 (2023-01-20)
------------------
* Build new msgs (`#117 <https://github.com/dfki-ric/mir_robot/issues/117>`_)
  Need to build new msg files. The files were added in a previous commit, but not added here for build and installation.
* Update MirMoveBase action to 2.10.3.1
* Update BMSData.msg to MiR software 2.13.2
  This is an "unsafe" change (it breaks compatibility with MiR versions <=
  2.13.2), but since BMSData is not published by the mir_driver currently,
  it should be ok.
* Add new messages added in MiR software 2.13.4.1
  This does not break compatibility with earlier versions (like 2.8.3.1),
  because these messages did not exist before.
  Actually, these messages were added in 2.10.3.1 (all other messages) and
  2.13.0.4 (ServiceResponseHeader.msg).
* Update Brake, Gripper + Height State msg to 2.13.4.1
  These are "unsafe" message changes (fields have been removed), but it
  should be ok because these are only used in HookExtendedStatus, and that
  message isn't forwarded by the mir_driver.
* Partially update messages to MiR software 2.13.4.1
  This only contains the "safe" changes, where fields were added. Also, it
  doesn't include the move of some messages to mir_hook_shared_interface,
  but keeps them here.
* Don't set cmake_policy CMP0048
* Contributors: Martin Günther, moooeeeep

1.1.6 (2022-06-02)
------------------
* Rename mir_100 -> mir
  This is in preparation of mir_250 support.
* Contributors: Martin Günther

1.1.5 (2022-02-11)
------------------

1.1.4 (2021-12-10)
------------------
* mir_msgs: Build all messages (`#98 <https://github.com/dfki-ric/mir_robot/issues/98>`_)
* Contributors: Martin Günther

1.1.3 (2021-06-11)
------------------
* Merge branch 'melodic-2.8' into noetic
* Update BMSData msg to MiR software 2.8.3.1
* Remove MirStatus
  This message was removed in MiR software 2.0 (Renamed to RobotStatus).
* Update mir_msgs to 2.8.2.2
* Contributors: Felix, Martin Günther

1.1.2 (2021-05-12)
------------------

1.1.1 (2021-02-11)
------------------
* Contributors: Martin Günther

1.1.0 (2020-06-30)
------------------
* Initial release into noetic
* Contributors: Martin Günther

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
