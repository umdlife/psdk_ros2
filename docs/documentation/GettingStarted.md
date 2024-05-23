# Getting started

## Configuration

There are two main configuration files needed to correctly run the `psdk_ros2` wrapper. 
1. **link_config.json**  *(default path: psdk_wrapper/cfg)* \
   A json file similar to the one used within the Payload-SDK samples which allows you to configure the type of connection/link you are using to connect to the DJI drone. This is related to the hardware you are using to connect to the drone.
2. **psdk_params.yaml** *(default path: psdk_wrapper/cfg)* \
   A yaml file which defines all ROS 2 parameters used by the wrapper. The information regarding the App configuration (e.g. app_id, app_ket, etc) can be configured here as a ROS 2 parameter.

### Hardware connection

Before launching the `psdk_ros2` wrapper, you must ensure that you are using the proper hardware connection to the DJI drone you are using. Various methods exist for establishing connections between DJI drones and companion computers. The approach may vary depending on the drone model and the companion computer in use. Below is a summary table illustrating each device port name and the DJI development kit that can be used. For the most recent updates, please refer to the latest version of the [table](https://developer.dji.com/doc/payload-sdk-tutorial/en/quick-start/drone-port.html).

| Aircraft       | Port Name                | Adapted Development Kit                                                 |
|----------------|--------------------------|-------------------------------------------------------------------------|
| Matrice 3D/3TD | E-Port                   | E-Port Development Kit                                                  |
|                | E-Port Lite              |     -                                                                   |
| FlyCart 30     | E-Port Lite              |  -                                                                      |
| M350 RTK       | E-Port                   | E-Port Development Kit / SDK Round Ribbon Cable                         |
|                | PSDK Port (Gimbal Port)  | SkyPort V2 Development Kit / SkyPort V2 Production Suit / DJI X-Port    |
| Mavic 3E/3T    | E-Port                   | E-Port Development Kit                                                  |
| Matrice 30/30T | E-Port                   | E-Port Development Kit / SDK Round Ribbon Cable / PSDK Mounting Bracket |
| M300 RTK       | OSDK Port                | E-Port Development Kit / SDK Round Ribbon Cable / OSDK Expansion Module |
|                | PSDK Port (Gimbal Port)  | SkyPort V1/V2 Development Kit / X-port /  SkyPort V1/V2 Production Suit |

Once you've established the hardware connection between your board and the DJI drone, you can configure the `psdk_ros2` wrapper to utilize that setup through the  *psdk_wrapper/cfg/link_config.json* file. One important parameter when filling in the link_config.json is to properly select what type of connection you are using. From all the possible configurations, the following have been (so far) tested and validated with the `psdk_ros2` wrapper.  

| Aircraft       | Port Name  | Adapted Development Kit |  psdk_ros2 link config       |
|----------------|------------|-------------------------|------------------------------|
| M350 RTK       | E-Port     | SDK Round Ribbon Cable  | use_uart_and_network_device  | 
| Matrice 30/30T | E-Port     | SDK Round Ribbon Cable  | use_uart_and_network_device  |
| M300 RTK       | OSDK Port  | OSDK Expansion Module   | use_uart_and_usb_bulk_device | 

The link configurations represent the different channels DJI uses to send information from the drone to 3rd party development platforms. In very generic terms, these channels are used for:
* UART serial: used to send commands to the drone and receive most of the information
* USB Bulk / Network device : used to retrive and send different cameras stream (e.g. main camera, FPV, perception) and perform media file management

Usually the serial interface is supported out-of-the-box by most systems, thus the basic functionalities of the psdk_ros2 wrapper are guaranteed. However, to interact with the different cameras on-board the drone, the USB bulk or Network device interface needs to be enabled on your computer. DJI offers a series of instructions and scripts to set-up the E-Port connection and enable the USB Bulk function for the Jetson Nano and Raspberry Pi 4B boards:
* Jetson Nano [instructions](https://developer.dji.com/doc/payload-sdk-tutorial/en/quick-start/quick-guide/jetson-nano.html) - Jetson Nano [scripts](https://terra-1-g.djicdn.com/71a7d383e71a4fb8887a310eb746b47f/psdk/e-port/usb-bulk-configuration-reference.zip)
  * If you want to enable the Ethernet interface, you can try to follow the instructions provided in the *readme.txt* file that comes within the scripts folder
* Raspberry 4B [instructions](https://developer.dji.com/doc/payload-sdk-tutorial/en/quick-start/quick-guide/raspberry-pi.html) - Raspberry 4B [scripts](https://sdk-forum.dji.net/hc/zh-cn/articles/10232604141465-M30%E5%BC%80%E5%8F%91-%E6%A0%91%E8%8E%93%E6%B4%BE4B%E9%85%8D%E7%BD%AEUSB-device-RNDIS-%E5%92%8C-BULK)

For any other board, you must make sure to enable either the USB Bulk or the Network interface to be able to access the camera stream via the psdk_ros2 wrapper. Please check the [Basic Concepts page](https://developer.dji.com/doc/payload-sdk-tutorial/en/quick-start/porting.html#basic-concepts) to know more about this topic. 

#### Example of DJI M300 - OKSD Port - OSDK Expansion Module connection 

This connection is compatible exclusively with the M300 RTK drone. Upon establishing the connection, two devices will be recognized by the computer, usually /tty/USB0 and /tty/ACM0. To maintain consistent device naming across sessions, refer to the 'Udev rules' section, which provides guidance on preventing device name alterations each time the devices are connected.

**Steps:**

1. Connect the FTDI adapter to the designated pins on the OSDK Exapansion module for serial communication
2. Use a mini-USB A to USB A cable (this cable is different depending on the ftdi module you use) to connect the FTDI adapter to the computer's USB 2.0 port
3. Connect the USB A to USB A from the OSDK Expansion module to the USB port of the computer
4. Connect the power cable from the OSDK Expansion module to supply power to the companion computer
5. Use the default *link_config.json* file to run the psdk_ros2 wrapper

<div style="background-color: #FFDDB8; padding: 10px; border: 0.2px solid ##FBFAFA;">
    <p style="margin: 0;"><strong> Caution:</strong>  The OSDK expansion module outputs 24V. Ensure that your companion computer can tolerate this voltage level. If not, you must use an appropriate step-down module to reduce the voltage to a safe level for your device.</p>
</div>
<div style="margin-bottom: 20px;"></div>


### ROS 2 Parameter configuration

The following parameters can be configured in the *psdk_wrapper/cfg/psdk_params.yaml* file:

| Parameter                     | Data Type | Default Value                      | Comments                                    |
| ------------------------------| --------- | ---------------------------------- | ------------------------------------------- |
| app_name                      | String    | -                                  | Add your App name                           |
| app_id                        | String    | -                                  | Add your App id                             |
| app_key                       | String    | -                                  | Add your App key                            |
| app_license                   | String    | -                                  | Add your App license                        |
| developer_account             | String    | -                                  | Add your developer account (not mandatory)  |
| baudrate                      | String    | 921600                             | -                                           |
| num_of_initialization_retries | Int       | 1                                  | Num of retries to init the PSDK app         |
| tf_frame_prefix               | String    | TF frame prefix                    | Add prefix before the frame name            |
| imu_frame                     | String    | "psdk_imu_link"                    | -                                           |
| body_frame                    | String    | "psdk_base_link"                   | -                                           |
| map_frame                     | String    | "psdk_map_enu"                     | -                                           |
| gimbal_frame                  | String    | "psdk_gimbal_link"                 | -                                           |
| camera_frame                  | String    | "psdk_camera_link"                 | -                                           |
| file download path            | String    | "/logs/media"                      | -                                           |
| mandatory_modules             |           |                                    |                                             |
| - telemetry                   | Bool      |  True                              | Trigger node failure, if module not loaded  |
| - flight_control              | Bool      |  True                              | Trigger node failure, if module not loaded  |
| - camera                      | Bool      |  False                             | Trigger node failure, if module not loaded  |
| - gimbal                      | Bool      |  False                             | Trigger node failure, if module not loaded  |
| - liveview                    | Bool      |  False                             | Trigger node failure, if module not loaded  |
| - hms                         | Bool      |  False                             | Trigger node failure, if module not loaded  |
| data_frequency                | Object    | -                                  | Options are: 1, 5, 10, 50, 100, 200, 400 Hz |
| - imu                         | Integer   | 100                                | -                                           |
| - attitude                    | Integer   | 100                                | -                                           |
| - acceleration                | Integer   | 50                                 | -                                           |
| - velocity                    | Integer   | 50                                 | -                                           |
| - angular_velocity            | Integer   | 100                                | -                                           |
| - position                    | Integer   | 50                                 | -                                           |
| - altitude                    | Integer   | 50                                 | -                                           |
| - gps_data                    | Integer   | 1                                  | -                                           |
| - rtk_data                    | Integer   | 1                                  | -                                           |
| - magnetometer                | Integer   | 50                                 | -                                           |
| - rc_channels_data            | Integer   | 1                                  | -                                           |
| - gimbal_data                 | Integer   | 1                                  | -                                           |
| - flight_status               | Integer   | 1                                  | -                                           |
| - battery_level               | Integer   | 1                                  | -                                           |
| - control_information         | Integer   | 1                                  | -                                           |
| - esc_data_frequency          | Integer   | 1                                  | -                                           |

## Udev rules

To avoid changing the device name each time you run the psdk application, you can use the following udev rules

```bash
# DJI Serial Comm
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="YourVendor", ATTRS{idProduct}=="YourProduct", MODE="0666", SYMLINK+="dji_serial"
# DJI Advanced Sensing
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="YourVendor", ATTRS{idProduct}=="YourProduct", MODE="0666", SYMLINK+="dji_advanced_sensing"
```
