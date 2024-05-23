# psdk_ros2

**psdk_ros2** is an open-source ROS 2 wrapper that brings DJI's [Payload-SDK](https://github.com/dji-sdk/Payload-SDK) libraries capabilities into the robotics ecosystem. 
You can find all the wrapper's documentation at [https://umdlife.github.io/psdk_ros2](https://umdlife.github.io/psdk_ros2/documentation/Introduction.html).
## Features 

The `psdk_ros2` wrapper currently supports the following features:

1. **Starting and Initializing PSDK Applications**: `psdk_ros2` allows you to initiate and configure a PSDK application from within ROS 2.
2. **Subscription to DJI Main Topics**: You can easily subscribe to the main topics provided by DJI, getting access to essential data and information.
3. **Frequency Control for Topic Subscription**: `psdk_ros2` offers the capability to set the frequency of topic subscriptions, allowing you to tailor data retrieval as per your requirements.
4. **Copter Position and Velocity Commands**: With `psdk_ros2`, you can conveniently send position and velocity commands to control a copter’s movement.
5. **Camera Parameters Management**: `psdk_ros2` provides functionalities to retrieve and update the main parameters of a mounted camera, giving you control over the camera settings.
6. **Gimbal Parameters Control**: You can manipulate gimbal parameters, adjusting the gimbal’s orientation and behavior.

## Compatibility 

The current version supports **PSDK v3.8.1** and it has been tested and built for **ROS 2 Humble**. For firmware and product compatibility, please check the page [DJI Developer - PSDK](https://developer.dji.com/doc/payload-sdk-tutorial/en/). 

In terms of hardware used for testing this wrapper, our primary testing platform has been the DJI M300 RTK drone, to which an external board has been connected via the OSDK expansion module. As for the payloads, H20 and H20T cameras have been tested. 

Please notice that this wrapper is still a work in progress. While we aim to make it as generic as possible, compatibility with different setups may require code modifications and testing to ensure proper functionality. Before deploying this wrapper with alternative hardware or configurations, we recommend thoroughly reviewing and understanding the codebase, configuration files, and documentation. 


## License

`psdk_ros2` wrapper is under the [Mozilla Public License Version 2.0](https://github.com/umdlife/psdk_ros2/blob/main/LICENSE.md). \
Please note that the [DJI Payload-SDK](https://github.com/dji-sdk/Payload-SDK) libraries which are needed to run this wrapper use MIT License. 
