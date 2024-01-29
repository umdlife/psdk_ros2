^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package psdk_interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
