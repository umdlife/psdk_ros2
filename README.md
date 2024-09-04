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


---

Here's the step-by-step instructions for running the PSDK wrapper as a non-root user without requiring a password.


## Running PSDK Wrapper as a Non-Root User

To run the PSDK wrapper as a non-root user, switch to the `non_root_user` branch and configure it according to the instructions provided in the [PSDK ROS 2 Wiki](https://umdlife.github.io/psdk_ros2/index.html). However, running the wrapper in this way will still prompt for a password. 

To avoid entering the password each time, follow the instructions below to modify the sudoers file so that specific commands can be executed without a password.

### Step-by-Step Instructions

1. **Backup the sudoers File**  
   Before making any changes, back up the existing sudoers file as a precaution:
   ```bash
   sudo cp /etc/sudoers /etc/sudoers.bak
   ```

2. **Edit the sudoers File**  
   You should never edit the sudoers file directly with a regular text editor. Instead, use `visudo`, which checks for syntax errors before saving:
   ```bash
   sudo visudo
   ```

3. **Modify the File**  
   Inside the editor, find the section that looks like this:
   ```bash
   # Allow members of group sudo to execute any command
   sudo ALL=(ALL:ALL) ALL
   ```
   Replace it with the following line to allow members of the sudo group to run commands without a password:
   ```bash
   sudo ALL=(ALL:ALL) NOPASSWD: ALL
   ```

4. **Save and Exit**  
   - If `visudo` opens with Vim (default editor), press `Esc`, type `:wq`, and press Enter to save and exit.
   - If `visudo` opens with Nano, press `Ctrl + O` to save, then Enter, and `Ctrl + X` to exit.


5. **Testing the Configuration**  
   Open a new terminal and run a command that typically requires a password, such as:
   ```bash
   sudo whoami
   ```
   If the command runs without prompting for a password, the configuration is correct. 

6. **Run the PSDK Wrapper**  
   Now, try launching the PSDK wrapper. It should initialize as a non-root user without requiring a password.
   ```bash
   ros2 launch psdk_wrapper wrapper.launch.py
   ```

---

This configuration allows members of the sudo group to execute commands without needing to enter a password, streamlining the process of running the PSDK wrapper. If you encounter any issues, revert to the backed-up sudoers file (`/etc/sudoers.bak`) and review the changes.