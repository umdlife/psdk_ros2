uint32 error_code           # Error code associated to HMS message
uint8 component_index       # Related to "payloadId" in the camera module
uint8 error_level
string ground_info          # Description of the error code if/when the drone is on the ground
string fly_info             # Description of the error code if/when the drone is in the air

uint8 MIN_HMS_ERROR_LEVEL = 0
uint8 MID_HMS_ERROR_LEVEL = 3
uint8 MAX_HMS_ERROR_LEVEL = 6

