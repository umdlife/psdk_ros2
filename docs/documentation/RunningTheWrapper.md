# Running the wrapper

## Debian packages for ROS 2 Humble

```bash
# Install debians
sudo apt install ros-humble-psdk-wrapper ros-humble-psdk-interfaces
source /opt/ros/humble/setup.bash

# Launch the node
# Default link_config_file_path = /opt/ros/humble/share/psdk_wrapper/cfg/link_config.json
# Default psdk_params_file_path = /opt/ros/humble/share/psdk_wrapper/cfg/psdk_params.yaml
# If using parameter and config files different than the default ones, you can point to them as:
ros2 launch psdk_wrapper wrapper.launch.py link_config_file_path:=/absolute/path/to/config.json psdk_params_file_path:=/absolute/path/to/params.yml
 
```

## Compile from source

To use the psdk_ros2 wrapper you will need to create a new workspace in which you clone both the wrapper as well as the Payload-SDK libraries. 

```bash
mkdir -p ~/psdk_ros2_ws/src
cd ~/psdk_ros2_ws/src
# Clone the psdk_ros2 wrapper
git clone https://github.com/umdlife/psdk_ros2.git

# Before building, check the Dependencies section and make sure you have everything installed
# You can also run rosdep to automatically install the dependencies
rosdep update
rosdep keys --from-paths . --ignore-src --rosdistro humble | \
  xargs rosdep resolve --rosdistro humble | \
  awk '/#apt/{getline; print}' > ./rosdep_requirements.txt
sudo apt install -y --no-install-recommends $(cat ./rosdep_requirements.txt) 

# Build the code
cd ~/psdk_ros2_ws
colcon build

# Launch the node
ros2 launch psdk_wrapper wrapper.launch.py

# Default link_config_file_path = psdk_wrapper/cfg/link_config.json
# Default psdk_params_file_path = psdk_wrapper/cfg/psdk_params.yml
# If using parameter and config files different than the default ones, you can point to them as:
ros2 launch psdk_wrapper wrapper.launch.py link_config_file_path:=/absolute/path/to/config.json psdk_params_file_path:=/absolute/path/to/params.yml

```

## Dependencies 

### ROS 2 packages

The following ROS 2 packages are needed to successfully build the wrapper:

* rclcpp
* rclcpp_lifecycle
* tf2
* tf2_ros
* sensor_msgs
* geometry_msgs
* std_msgs
* nav_msgs
* std_srvs

### Other libraries

The following libraries are needed to enable the access to USB devices and handling the video streaming:

* libusb-1.0-0-dev
* libopus-dev 
* ffmpeg 
* libavcodec-dev 
* libavformat-dev 
* libavfilter-dev

The following library is used to work with JSON:
* nlohmann-json-dev
