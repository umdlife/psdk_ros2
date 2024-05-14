# Introduction

`psdk_ros2` wrapper is an open-source project that integrates the DJI's [Payload-SDK libraries](https://github.com/dji-sdk/Payload-SDK) with ROS 2 (Robot Operating System 2) ecosystem. 

## Key features

The `psdk_ros2` wrapper currently supports the following features:

1. **Starting and Initializing PSDK Applications**: `psdk_ros2` allows you to initiate and configure a PSDK application from within ROS 2.

2. **Subscription to DJI Main Topics**: You can easily subscribe to the main topics provided by DJI, getting access to essential data and information.

3. **Frequency Control for Topic Subscription**: `psdk_ros2` offers the capability to set the frequency of topic subscriptions, allowing you to tailor data retrieval as per your requirements.

4. **Copter Position and Velocity Commands**: With `psdk_ros2`, you can conveniently send position and velocity commands to control a copter's movement.

5. **Camera Management**: `psdk_ros2` provides functionalities to retrieve and update the main parameters of a mounted camera, giving you control over the camera settings. Moreover, you can take photos, videos and obtain the stream on a ROS 2 topic. 

6. **Gimbal Control**: You can manipulate gimbal parameters, adjusting the gimbal's orientation and behavior.

7. **Health management system**: All DJI notifications and errors seen on the RC can be retrieved over a ROS 2 topic. 



<div style="background-color: #D6EAF8; padding: 10px; border: 1px solid ##FBFAFA;">
    <p style="margin: 0;"><strong> Note:</strong>  The PSDK libraries offer many other capabilities. If you are interested in enhancing the wrapper and adding further functionalities, please visit our Contribute page to discover how you can contribute to the community.</p>
</div>

<div style="margin-bottom: 20px;"></div>

## Compatibility

The current version supports **PSDK v3.8.1** and it has been tested and built for **ROS 2 Humble**. For firmware and product compatibility, please check the page [DJI Developer - PSDK](https://developer.dji.com/doc/payload-sdk-tutorial/en/).


## Important Notice 

<div style="background-color: #FFDDB8; padding: 10px; border: 0.2px solid ##FBFAFA;">
    <p style="margin: 0;"><strong> Caution:</strong>  This code has not undergone extensive testing with real hardware. Users are advised to exercise caution when using this repository.</p>
</div>

<div style="margin-bottom: 20px;"></div>

While we strive to provide reliable and functional code, it's important to note that the code in this repository is still a work in progress. As a result, potential bugs or compatibility issues might arise when using this code with specific hardware configurations.

We encourage users to:

- Test the code in controlled environments before deploying it in critical or real-world scenarios.
- Proceed with caution and monitor the system closely if integrating this code into operational hardware.
- Provide feedback, report issues and open pull requests. Your input will help improve the reliability of this repository.

Remember that all users assume the responsibility for assessing the code's suitability for their own applications and hardware.
