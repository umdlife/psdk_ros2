# The psdk_ros2 wrapper

This brief overview highlights the primary components and their roles within the `psdk_ros2` wrapper's code structure.

├── **psdk_interfaces** : *provides the main interfaces (msg, srv)* \
├── **psdk_wrapper** : *main wrapper code* 

Within the `psdk_wrapper` folder:

└── **psdk_wrapper** \
├── **[...]**	 \
├── **src**      \
&nbsp;&nbsp;&nbsp;├── **3rdparty** : *Contains essential libraries sourced from DJI Payload-SDK for FCU interaction* \
&nbsp;&nbsp;&nbsp;├── main.cpp                                                                                        \
&nbsp;&nbsp;&nbsp;├── psdk_wrapper.cpp \
&nbsp;&nbsp;&nbsp;└── **modules** : *Encapsulates specific functionalities of the wrapper.*   \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;├── telemetry.cpp  \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;├── flight_control.cpp \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;├── camera.cpp \
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;├── gimbal.cpp 
    		

## Modules

### Telemetry

This module subscribes to the main PSDK data exposed and publishes it on ROS 2 topics. Several topic categories have been defined to which a certain publishing frequency can be set via the configuration file. Nonetheless, some topics have a publishing frequency restricted from DJI, so if the frequency set does not correspond to the one handled in the DJI PSDK libraries, an error message will appear. 

Please note, that the GPS position is such a topic. From what we observed, it is only published at 1 Hz. 

### Flight control 

### Camera 

### Gimbal

