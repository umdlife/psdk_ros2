std_msgs/Header header
uint8 air_connection                    # Sky side is connected, i.e., receiver is connected to FC
uint8 ground_connection                 # Ground side is connected, i.e., RC is on and connected to FC
uint8 app_connection                    # Mobile App is connected to RC
uint8 air_or_ground_disconnected        # 0 if sky or ground side is disconnected for 3 seconds