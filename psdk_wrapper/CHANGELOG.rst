^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package psdk_wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2024-02-20)
------------------
* Merge pull request `#51 <https://github.com/umdlife/psdk_ros2/issues/51>`_ from umdlife/feat/hms-support
  Integration of Health Monitoring System (HMS) as a module
* Merge pull request `#59 <https://github.com/umdlife/psdk_ros2/issues/59>`_ from umdlife/extend_single_battery_info
  Extend single battery info
* Merge pull request `#57 <https://github.com/umdlife/psdk_ros2/issues/57>`_ from umdlife/feat/single_battery_info
  Adding topic for reading battery info
* Contributors: Stevedan, Victor Massagué Respall, amoramar, biancabnd, marta

1.0.0 (2024-02-14)
------------------
* Merge pull request `#34 <https://github.com/umdlife/psdk_ros2/issues/34>`_ from umdlife/feat/upgrade-to-psdk-v3.8
  Feat/upgrade to psdk v3.8
* Merge pull request `#44 <https://github.com/umdlife/psdk_ros2/issues/44>`_ from RPS98/feat/upgrade-to-psdk-v3.8
  Add config files as launcher configs
* Set camera, gimbal and streaming modules as non mandatory
* Merge pull request `#49 <https://github.com/umdlife/psdk_ros2/issues/49>`_ from umdlife/feat/parametrise-retry-num
  Make the number of init retries as a ros param
* Upgrade wrapper to be compatible with DJI PSDK v3.8
* Contributors: Rafael Perez-Segui, Sergi Grau Moya, Victor Massagué Respall, biancabnd

0.0.5 (2024-02-05)
------------------
* Move lifecycle interfaces to be public, also add rclcpp::shutdown() to finish cleanly the node when shutting down
* Contributors: Victor Massagué Respall, biancabnd, sergigraum

0.0.4 (2024-01-29)
------------------
* Merge pull request `#45 <https://github.com/umdlife/psdk_ros2/issues/45>`_ from umdlife/feat/prepare-ros-index
  Prepare for ROS Index
* Reorder method call in lifecycle to avoid node crashes + increase nr. of retries for psdk init
* Update maintainers
* Contributors: Victor Massagué Respall, biancabnd, vicmassy

0.0.3 (2024-01-24)
------------------
* Merge pull request `#35 <https://github.com/umdlife/psdk_ros2/issues/35>`_ from umdlife/feat/stream-publisher
  Enable encoded and decoded stream publishing
* Add license tag to package.xml
* Add missing SensorDataQoS
* Enable the user to choose among raw or decoded streaming
* Publish encoded images
* Use default QoS setting from stream publisher
* Enable IPC + change camera publisher QoS to SensorData
* Change streaming publishing method
* Contributors: Victor Massagué Respall, amoramar, biancabnd, vicmassy

0.0.2 (2024-01-15)
------------------
* Add permisions to open release from CI
* Merge pull request `#37 <https://github.com/umdlife/psdk_ros2/issues/37>`_ from umdlife/hotfix/add-missing-declare
  Add missing declare + lower control information topic frequency
* Merge pull request `#33 <https://github.com/umdlife/psdk_ros2/issues/33>`_ from umdlife/feat/add-data-topics
  Additional topics
* Correct Doxygen documentation
* Add altitude topics
  Add max retry nr
* Add obstacle avoidance data + retry strategy for initialization function
* Contributors: Sergi Grau Moya, Victor Massagué Respall, amoramar, biancabnd, vicmassy

0.0.1 (2023-09-28)
------------------
* Merge pull request `#17 <https://github.com/umdlife/psdk_ros2/issues/17>`_ from umdlife/feat/add-additional-status-info
  Add additional status info
* Add home point and control mode info
* Add RTK connection status publisher
* Add RC connection status publisher
* Contributors: Victor Massagué Respall, biancabnd

0.0.0 (2023-09-26)
------------------
* Merge pull request `#15 <https://github.com/umdlife/psdk_ros2/issues/15>`_ from umdlife/feat/add-static-transforms
  Add static and dynamic transforms supporting M300 copter + H20 camera
* Change frame id of imu topic
* Fix error on getting camera source requested by user
* Set optical frame id to streaming topic
* Fix gimbal angles + add dynamic TF
* First version of static transform publisher
* Temporally remove the z health check
* Change gimbal angles reference frame for control
* Separate gps_position_fused freq of other gps data
* Account for floating point division in battery msg
* Use standard battery msgs instead of custom one
* Change acceleration from Vector3 to AccelStamped
* Renamings, fix errors, enhanced API documentation
* Add acceleration ground, body and raw topics
* Add angular rate topics
* Fix error in set_local_pose_ref srv
* Add set_local_pose_ref\_ srv
* Convert gps fused msg to standard sensor_msgs NavsatFix
* Convert gimbal command from ENU to NED
* Add fpv camera streaming on separated ros 2 topic
* Renamings + additional documentation added for all services and subscribers
* Add set/get camera aperture function
* Unify init + deinit strategy across modules
* Remove 3rdparty code with no modifications from psdk_ros2
* Add init and deinit function for streaming
* Add 3rdparty libs cmake + remove comments
* Convert gimbal angles to rad and ENU + fix spelling errors
* Renamig of method for vo_position + small comment fix
* Rename aircraft_status to display_mode + update docs
* Update documentation + fix error topic naming + add additional comments
* Add namespace to launch file + rename tag before topics/service
* Update docs + add configuration to see private members in api
* Remove nav2_util lifecycle + add launch file for wrapper node
* Add launch file
* Unify error code format
* Add documentation pipeline + Copyright statement
* Add Mozilla license + Update README.md
* Add frames as params + improve comments
* Merge pull request `#2 <https://github.com/umdlife/psdk_ros2/issues/2>`_ from umdlife/feat/psdk-sensors
  Feat/psdk sensors
* gimbal rotation is subscriber
* add streaming path as parameter
* angles in rad instead of deg
* remove comments
* PSDK sensors working
* Merge branch 'main' into feat/psdk-sensors
* Merge pull request `#3 <https://github.com/umdlife/psdk_ros2/issues/3>`_ from umdlife/feat/psdk-core-UP-2096
  PSDK core functions + subscribers + basic flight control
* Contributors: UMLDev Clang Robot, Victor Massagué Respall, biancabnd, lidia
