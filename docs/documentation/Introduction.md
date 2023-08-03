# Introduction

`psdk_ros2` wrapper is an open-source project that integrates the DJI's [Payload-SDK libraries](https://github.com/dji-sdk/Payload-SDK) with ROS2 (Robot Operating System 2) ecosystem. 

## Key features

The `psdk_ros2` wrapper currently supports the following features:

1. **Starting and Initializing PSDK Applications**: `psdk_ros2` allows you to initiate and configure a PSDK application from within ROS 2.

2. **Subscription to DJI Main Topics**: You can easily subscribe to the main topics provided by DJI, getting access to essential data and information.

3. **Frequency Control for Topic Subscription**: `psdk_ros2` offers the capability to set the frequency of topic subscriptions, allowing you to tailor data retrieval as per your requirements.

4. **Copter Position and Velocity Commands**: With `psdk_ros2`, you can conveniently send position and velocity commands to control a copter's movement.

5. **Camera Parameters Management**: `psdk_ros2` provides functionalities to retrieve and update the main parameters of a mounted camera, giving you control over the camera settings.

6. **Gimbal Parameters Control**: You can manipulate gimbal parameters, adjusting the gimbal's orientation and behavior.

7. **Camera Streaming Visualization**: `psdk_ros2` facilitates the visualization of camera streaming on ROS 2 topics, allowing you to view live feeds.


<div style="background-color: #D6EAF8; padding: 10px; border: 1px solid #c3d9ff;">
    <p style="margin: 0;"><strong> Note:</strong>  The PSDK libraries offer many other capabilities. If you are interested in enhancing the wrapper and adding further functionalities, please visit our Contribute page to discover how you can contribute to the community.</p>
</div>


## Compatibility

The current version supports **PSDK v3.5** and it has been tested and build for **ROS 2 Humble**. For firmware and product compatibility, please check the page [DJI Developer - PSDK](https://developer.dji.com/doc/payload-sdk-tutorial/en/).

