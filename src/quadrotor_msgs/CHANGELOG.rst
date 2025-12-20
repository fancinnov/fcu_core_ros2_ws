^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package quadrotor_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.19 (2019-02-11)
-------------------
* Add CascadedCommandVector
  Used to send vector cascaded commands from MPC position to MPC attitude.
* Contributors: Alex Spitzer

0.0.18 (2019-02-11)
-------------------
* Merge branch 'feature/explicit_epc' into 'develop'
  Added l1 attitude debug messages coming from firmware
  See merge request msgs/quadrotor_msgs!21
* Correcting feature/explicit_epc branch
* Adding L1 Attitude debug messages coming from Firmware
* Merge branch 'feature/explicit_epc' into 'develop'
  battery msg name change from BatteryVoltage to BatteryStatus
  See merge request msgs/quadrotor_msgs!20
* changing battery related messages from BatteryVoltage to BatteryStatus
* Merge branch 'develop' of gitlab.rislab.org:msgs/quadrotor_msgs into develop
* Merge branch 'feature/explicit_epc' into 'develop'
  header timestamp and cell_id_epc msg addition in battery_status
  See merge request msgs/quadrotor_msgs!19
* header timestamp and cell_id_epc msg addition in battery_status
* Merge branch 'develop' of gitlab.rislab.org:msgs/quadrotor_msgs into develop
* Merge branch 'feature/explicit_epc' into 'develop'
  adding current msg float
  See merge request msgs/quadrotor_msgs!18
* adding current msg float
* adding current receive float32
* Merge branch 'feature/state_estimate_validity' into develop
* Merge branch 'feature/publish_desired_hod' into 'develop'
  Add jerk_des and snap_des to TrackingError.msg
  See merge request msgs/quadrotor_msgs!17
* Add jerk_des and snap_des to TrackingError.msg
* Merge branch 'feature/tracking_error_semantic_comment' into 'develop'
  Add comment to TrackingError.msg indicating that err is actual - desired
  See merge request !16
* Add comment to TrackingError.msg indicating that err is actual - desired
* add RelativeVerticalMotion message type
* merge feature/0.17_vision_rc
* Use size_t for mx array size to match API change.
* Append IPC and ipc_bridge include paths to Matlab c flags
* remove unused AccelDisturbance message type
* add new FSM modes DRIFT_AUTO and DRIFT_TELE
* add message type to indicate validity of states in state estimate
* Merge branch 'feature/pointsArray' of gitlab.rasl.ri.cmu.edu:msgs/quadrotor_msgs into feature/0.17_vision_rc
* Add a PointArray msg for publishing arrays of points
* Merge branch 'feature/spline_array' of gitlab.rasl.ri.cmu.edu:msgs/quadrotor_msgs into feature/0.17_vision_rc
* Adding num_splines to SplineArray.msg
* Adding SplineArray.msg
* Contributors: Alex Spitzer, John Yao, Mosam Dabhi, Nathan Michael, Xuning Yang, dnovichman

0.0.17 (2017-03-06)
-------------------
* Adding StampedWaypoint msg
* add ScalarWithVarianceStamped message
* increase array size for rotor force command message from 4 to 6
* increase array size of rpm and pwm messages from 4 to 6
* added rotor force command message to aid CASM computation
* message support for concatenating to spline and popping and concatenating to spline
* Changed order field to variable size array in Spline.msg
* Removed dt and made order an array in Spline msg
* Added pos_des, vel_des, yaw_des, yawdot_des fields to TrackingError.msg
* adding AccelDisturbance message
* Contributors: John Yao, Vishnu Desaraju, Wennie Tabib, Xuning Yang

0.0.16 (2016-04-16)
-------------------
* Add tele-operation mode to FSM mode msg
* Update position command to include three heading fields (psi, dpsi, ddpsi)
* IPC support for position cmd and gains msgs
* Position command and gains msgs
* OdometryArray msg and IPC support
* Updating ipc msgs to support std_msgs Header (vs deprecated rosgraph_msgs)
* ipc for cascaded gains msg
* Add cascaded cmd gain msg
* Contributors: Nathan Michael, eacappo

0.0.15 (2016-02-11)
-------------------
* message to log power with timestamp
* move LidarLiteOutput message from lidar_lite to quadrotor_msgs
* add z-component of velocity for lidar-lite observation
* adding message for PX4Flow body velocity estimate
* Removing deprecated msgs. Addresses issue #2
* added std_msgs to package.xml
* modification of the "cascaded_command" message to include angular acceleration
* Contributors: John Yao, Nathan Michael, Wennie Tabib, eacappo

0.0.14 (2015-11-05)
-------------------
* Merge branch 'bugfix/add_std_msgs_dependency' into 'develop'
  Add dependency on std_msgs.
  Closes #1.
  See merge request !4
* Add dependency on std_msgs. Closes #1.
* Contributors: Nathan Michael

0.0.13 (2015-08-04)
-------------------
* Merge branch 'feature/fsm_mode' into 'develop'
  Feature/fsm mode
  See merge request !3
* Updated FSMMode msg to match mdes in FSM
* Contributors: Nathan Michael, Vishnu Desaraju

0.0.12 (2015-07-03)
-------------------
* Added TrackingError msg
* Added FSMMode message
* Contributors: Vishnu Desaraju

0.0.11 (2015-06-12)
-------------------
* Adding current_heading entry to enable yaw control from external reference
* Adding support for new cascaded command
* Add include directory to CMakeLists
* Adding support for RPMCommand message
* add header for the GPIOTime message
* GPIOTime msg definition
* Contributors: Nathan Michael, Zheng Rong

0.0.10 (2014-11-04)
-------------------
* Merge branch 'feature/spline_msg' into 'develop'
  Feature/spline msg
  See merge request !2
* Merge branch 'develop' into feature/spline_msg
  Conflicts:
  CMakeLists.txt
* Added new Spline message type to send trajectory coefficients
* Contributors: Nathan Michael, Vishnu Desaraju

0.0.9 (2014-10-24)
------------------
* Merge branch 'feature/local_odometry' into 'develop'
  Feature/local odometry
  See merge request !1
* added local odometry message type
* Contributors: John Yao, Nathan Michael

0.0.8 (2014-09-16)
------------------
* added message type for transient trim observer information
* add u_Mz and u_acc fields to StaticTrimInfo message
* Removing AltitudeInfo messsage and absorbing it into AltitudeObservation
* added message type for static trim estimator information
* added AltitudeInfo message
* add HeightDelta message
* Adding AltitudeObservation.msg (previously in altitude_pf)
* Contributors: Erik Nelson, John Yao, Nathan Michael

0.0.7 (2014-05-05)
------------------
* Changes to handling of PDCommands and yaw_delta
* Contributors: Nathan Michael

0.0.6 (2014-04-07)
------------------
* Adding support for custom BatteryStatus msg
* Merge branch 'develop' of nmichael.frc.ri.cmu.edu:msgs/quadrotor_msgs into develop
* Updating PDCommand support and renaming Command to SO3Command
* Contributors: Nathan Michael

0.0.5 (2014-03-26)
------------------
* Fix thrust setting and incorrect header include
* Adding support for PDCommands
* Contributors: Nathan Michael

0.0.4 (2014-02-25)
------------------
* Quieting output if IPC is not found
* Added missing IPC header include variable
* Fixing incorrect call to find_package without specifying components
* Updates to support rework of IPC bridge
* New PWMCommand and IPC bridge interface
* Updating license tag to specify GPLv2
* Contributors: Nathan Michael

0.0.3 (2014-01-14)
------------------
* Adding GPLv2 license and documentation base
* Contributors: Nathan Michael

0.0.2 (2014-01-13)
------------------
* Moving status msg to quadrotor_msgs package
* Contributors: Nathan Michael

0.0.1 (2014-01-13)
------------------
* Updating to remove corrections term and vector3 instead of float64 arrays for gains
* Moving to hydro
* Template hydro files
* Removing in place turn as it is accomplished by hover
* Initial commit
* Contributors: Nathan Michael
