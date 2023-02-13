std_msgs/Header header
uint8 control_mode         # see CtlrMode in dji_status.hpp
uint8 device_status        # 0->rc  1->app  4->serial 
uint8 flight_status        # 1->opensd  0->close 
uint8 vrc_status
uint8 reserved