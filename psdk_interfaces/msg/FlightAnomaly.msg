std_msgs/Header header
uint32 impact_in_air                     # 0: No impact,                      1: _impact happens in Air 
uint32 random_fly                       # 0: Normal,                         1: Randomly fly in GPS mode without stick input
uint32 height_ctrl_fail                  # 0: Height control normal,          1: Height control failed 
uint32 roll_pitch_ctrl_fail               # 0: Tilt control normal,            1: Tilt control failed 
uint32 yaw_ctrl_fail                     # 0: Yaw control normal,             1: Yaw control failed 
uint32 aircraft_is_falling              # 0: Aircraft is not falling,        1: Aircraft is falling 
uint32 strong_wind_level1               # 0: wind is under big wind level 1, 1: wind is stronger than  big wind level 1
uint32 strong_wind_level2               # 0: wind is under big wind level 2, 1: wind is stronger than  big wind level 2
uint32 compass_installation_error       # 0: Compass install right,          1: Compass install error 
uint32 imu_installation_error           # 0: IMU install right,              1: IMU install error 
uint32 esc_temperature_high             # 0: ESC temperature is normal,      1: ESC temperature is high 
uint32 at_least_one_esc_disconnected     # 0: No ESC disconnected,            1: At least one ESC is disconnected 
uint32 gps_yaw_error                    # 0: No GPS yaw error,               1: GPS yaw error 
uint32 reserved