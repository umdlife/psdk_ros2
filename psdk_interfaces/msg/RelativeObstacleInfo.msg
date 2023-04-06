std_msgs/Header header

float32 down               # distance from obstacle (m) 
float32 front              # distance from obstacle (m) 
float32 right              # distance from obstacle (m) 
float32 back               # distance from obstacle (m) 
float32 left               # distance from obstacle (m) 
float32 up                 # distance from obstacle (m) 
uint8 down_health          # Down sensor flag: 0 - not working, 1 - working 
uint8 front_health         # Front sensor flag: 0 - not working, 1 - working 
uint8 right_health         # Right sensor flag: 0 - not working, 1 - working 
uint8 back_health          # Back sensor flag: 0 - not working, 1 - working 
uint8 left_health          # Left sensor flag: 0 - not working, 1 - working 
uint8 up_health            # Up sensor health flag: 0 - not working, 1 - working 
uint8 reserved              # Reserved sensor health flag