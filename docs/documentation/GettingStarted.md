 # Getting started

To use the psdk_ros2 wrapper you will need to create a new workspace in which you clone both the wrapper as well as the Payload-SDK libraries. 

```bash
mkdir -p ~/psdk_ros2_ws/src
cd ~/psdk_ros2_ws/src
# Clone the psdk_ros2 wrapper and the Payload-SDK 
git clone https://github.com/umdlife/psdk_ros2.git
git clone https://github.com/dji-sdk/Payload-SDK.git

# Build the code
cd ~/psdk_ros2_ws
colcon build --packages-skip entry	# Skip the build corresponding to DJI PSDK sample code

# Launch the node
ros2 launch psdk_wrapper wrapper.launch

```



## Configuration

The following parameters can be configured in the *psdk_wrapper/cfg/psdk_params.yaml* file:

| Parameter             | Data Type | Default Value                      | Comments                                    |
| --------------------- | --------- | ---------------------------------- | ------------------------------------------- |
| app_name              | String    | -                                  | Add your App name                           |
| app_id                | String    | -                                  | Add your App id                             |
| app_key               | String    | -                                  | Add your App key                            |
| app_license           | String    | -                                  | Add your App license                        |
| developer_account     | String    | -                                  | Add your developer account (not mandatory)  |
| baudrate              | String    | 921600                             | -                                           |
| hardware_connection   | String    | "DJI_USE_UART_AND_USB_BULK_DEVICE" | Depends on your connection method           |
| uart_dev_1            | String    | "/dev/dji_serial"                  | As defined in udev rules                    |
| uart_dev_2            | String    | "/dev/dji_advanced_sensing"        | As defined in udev rules                    |
| imu_frame             | String    | "psdk_imu_link"                    | -                                           |
| body_frame            | String    | "psdk_base_link"                   | -                                           |
| map_frame             | String    | "psdk_map_enu"                     | -                                           |
| gimbal_frame          | String    | "psdk_gimbal_link"                 | -                                           |
| data_frequency        | Object    | -                                  | Options are: 1, 5, 10, 50, 100, 200, 400 Hz |
| - imu                 | Integer   | 100                                | -                                           |
| - attitude            | Integer   | 100                                | -                                           |
| - acceleration        | Integer   | 50                                 | -                                           |
| - velocity            | Integer   | 50                                 | -                                           |
| - angular_velocity    | Integer   | 100                                | -                                           |
| - position            | Integer   | 50                                 | -                                           |
| - gps_data            | Integer   | 1                                  | -                                           |
| - rtk_data            | Integer   | 1                                  | -                                           |
| - magnetometer        | Integer   | 50                                 | -                                           |
| - rc_channels_data    | Integer   | 1                                  | -                                           |
| - gimbal_data         | Integer   | 1                                  | -                                           |
| - flight_status       | Integer   | 1                                  | -                                           |
| - battery_level       | Integer   | 1                                  | -                                           |
| - control_information | Integer   | 50                                 | -                                           |



## Udev rules

To avoid changing the device name each time you run the psdk application, you can use the following udev rules

```bash
# DJI Serial Comm
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="YourVendor", ATTRS{idProduct}=="YourProduct", MODE="0666", SYMLINK+="dji_serial"
# DJI Advanced Sensing
SUBSYSTEM=="tty", SUBSYSTEMS=="usb", ATTRS{idVendor}=="YourVendor", ATTRS{idProduct}=="YourProduct", MODE="0666", SYMLINK+="dji_advanced_sensing"
```



## Dependencies 

* DJI Payload-SDK

  The libraries responsible for facilitating the direct interaction between the `psdk_ros2` wrapper and the real Flight Control Unit (FCU) of a DJI drone have been sourced from the DJI Payload-SDK's original codebase.  These libraries has been contained within the `3rdparty` folder. 

  The main reason to include these libraries within the wrapper source code is t is to facilitate the dynamic configuration of UART device names used to establish the connection between the PSDK code and the FCU.  In the standard PSDK sample code, these device names are defined within a header file. However, this wrapper introduces the capability to configure these devices via a configuration file.



# 