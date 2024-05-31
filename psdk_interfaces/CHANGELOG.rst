^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package psdk_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.3.0 (2024-05-31)
------------------
* Separate each module with an individual thread
* Contributors: Rafael Perez-Segui, biancabnd

1.2.1 (2024-05-24)
------------------
* Merge pull request `#101 <https://github.com/umdlife/psdk_ros2/issues/101>`_ from umdlife/hotfix/build-farm
  Add action_msgs as dependency
* Contributors: biancabnd, vicmassy

1.2.0 (2024-05-23)
------------------
* Merge pull request `#81 <https://github.com/umdlife/psdk_ros2/issues/81>`_ from umdlife/feat/sd_images
  SD card functions
* Merge pull request `#76 <https://github.com/umdlife/psdk_ros2/issues/76>`_ from umdlife/feat/psdk-3.8.1
  Upgrade to Payload-SDK v3.8.1
* Contributors: DominikWawak, Victor Massagué Respall, biancabnd

1.1.1 (2024-03-27)
------------------
* Merge pull request `#68 <https://github.com/umdlife/psdk_ros2/issues/68>`_ from umdlife/hotfix/motors-stop
  Add ESC data
* Merge remote-tracking branch 'origin/main' into add_namespace_to_tf
* Contributors: DominikWawak, Rafael Perez-Segui, biancabnd

1.1.0 (2024-02-20)
------------------
* Merge pull request `#51 <https://github.com/umdlife/psdk_ros2/issues/51>`_ from umdlife/feat/hms-support
  Integration of Health Monitoring System (HMS) as a module
* Merge pull request `#59 <https://github.com/umdlife/psdk_ros2/issues/59>`_ from umdlife/extend_single_battery_info
  Extend single battery info
* Merge pull request `#57 <https://github.com/umdlife/psdk_ros2/issues/57>`_ from umdlife/feat/single_battery_info
  Add new message types HmsInfoMsg, HmsInfoTable
* Contributors: Stevedan, Victor Massagué Respall, amoramar, biancabnd, marta

1.0.0 (2024-02-14)
------------------
* Merge pull request `#34 <https://github.com/umdlife/psdk_ros2/issues/34>`_ from umdlife/feat/upgrade-to-psdk-v3.8
  Feat/upgrade to psdk v3.8
* Upgrade wrapper to be compatible with DJI PSDK v3.8
* Contributors: Victor Massagué Respall, biancabnd

0.0.5 (2024-02-05)
------------------
* Merge pull request `#48 <https://github.com/umdlife/psdk_ros2/issues/48>`_ from umdlife/hotfix/update_documentation
  Unify build and deploy documentation workflow
* Contributors: biancabnd

0.0.4 (2024-01-29)
------------------
* Update maintainers
* Contributors: vicmassy

0.0.3 (2024-01-24)
------------------
* Merge pull request `#35 <https://github.com/umdlife/psdk_ros2/issues/35>`_ from umdlife/feat/stream-publisher
  Enable encoded and decoded stream publishing
* Add license tag to package.xml
* Enable the user to choose among raw or decoded streaming
* Contributors: Victor Massagué Respall, biancabnd, vicmassy

0.0.2 (2024-01-15)
------------------
* Merge pull request `#33 <https://github.com/umdlife/psdk_ros2/issues/33>`_ from umdlife/feat/add-data-topics
  Additional topics
* Add altitude topics
  Add max retry
* Contributors: Sergi Grau Moya, biancabnd, vicmassy

0.0.1 (2023-09-28)
------------------
* Merge pull request `#17 <https://github.com/umdlife/psdk_ros2/issues/17>`_ from umdlife/feat/add-additional-status-info
  Add additional status info
* Add home point and control mode info
* Add RC connection status publisher
* Contributors: Victor Massagué Respall, biancabnd

0.0.0 (2023-09-26)
------------------
* Merge pull request `#7 <https://github.com/umdlife/psdk_ros2/issues/7>`_ from umdlife/feat/open-source-prep
  Open source preparations
* Use standard battery msgs instead of custom one
* Convert gps fused msg to standard sensor_msgs NavsatFix
* Convert gimbal command from ENU to NED
* Renamings + additional documentation added for all services and subscribers
* Add set/get camera aperture function
* Add streaming interface
* Add comments to methods/ros publishers + convert gimbal angles to rad and ENU + fix spelling errors
* Rename aircraft_status to display_mode + update docs
* Merge pull request `#2 <https://github.com/umdlife/psdk_ros2/issues/2>`_ from umdlife/feat/psdk-sensors
  Feat/psdk sensors
* removing streaming
* gimbal rotation is subscriber
* changes to address PR comments
* angles in rad instead of deg
* Merge branch 'main' into feat/psdk-sensors
* Merge pull request `#3 <https://github.com/umdlife/psdk_ros2/issues/3>`_ from umdlife/feat/psdk-core-UP-2096
  PSDK core functions + subscribers + basic flight control
* Rename folders and classes
* Contributors: Victor Massagué Respall, biancabnd, lidia
