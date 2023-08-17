std_msgs/Header header
uint32 mount_status            # 1 - gimbal mounted, 0 - gimbal not mounted
uint32 is_busy
uint32 pitch_limited           # 1 - axis reached limit, 0 - no 
uint32 roll_limited            # 1 - axis reached limit, 0 - no 
uint32 yaw_limited             # 1 - axis reached limit, 0 - no 
uint32 calibrating            # 1 - calibrating, 0 - no 
uint32 prev_calibration_result # 1 - success, 0 - fail 
uint32 installed_direction     # 1 - reversed for OSMO, 0 - normal 
uint32 disabled_mvo
uint32 gear_show_unable
uint32 gyro_falut              # 1 - fault, 0 - normal 
uint32 esc_pitch_status         # 1 - Pitch data is normal, 0 - fault 
uint32 esc_roll_status          # 1 - Roll data is normal, 0 - fault 
uint32 esc_yaw_status           # 1 - Yaw data is normal , 0 - fault 
uint32 drone_data_recv          # 1 - normal , 0 - at fault 
uint32 init_unfinished         # 1 - init complete, 0 - not complete 
uint32 fw_updating             # 1 - updating, 0 - not updating 