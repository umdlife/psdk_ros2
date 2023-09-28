std_msgs/Header header
uint8 control_mode      # The modes in which the aircraft is being controlled (control loops being applied on horizontal, vertical and yaw axes of the aircraft)
uint8 device_mode       # 0->RC  1->MSDK  4->PSDK
uint8 control_auth      # 1->Has control authority 0->No control authority