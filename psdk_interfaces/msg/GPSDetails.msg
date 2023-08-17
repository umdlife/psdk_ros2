std_msgs/Header header
float32 horizontal_dop                             # Horizontal dilution of precision, unit: 0.01, eg: 100 = 1.00, <1: ideal, 1-2: excellent, 2-5: good, 5-10: moderate, 10-20: fair, >20: poor.
float32 position_dop                               # Position dilution of precision, unit: 0.01, eg: 100 = 1.00, <1: ideal, 1-2: excellent, 2-5: good, 5-10: moderate, 10-20: fair, >20: poor.

uint16 GPS_FIX_STATE_NO_FIX = 0                    # GPS position has not been fixed. 
uint16 GPS_FIX_STATE_DEAD_RECKONING_ONLY = 1       # GPS position is dead reckoned only. 
uint16 GPS_FIX_STATE_2D_FIX = 2                    # The horizontal position with latitude/longitude (or northing/easting or X/Y) is fixed. 
uint16 GPS_FIX_STATE_3D_FIX = 3                    # The horizontal and vertical position with latitude/longitude/altitude (northing/easting/altitude or X/Y/Z) is fixed. 
uint16 GPS_FIX_STATE_GPS_PLUS_DEAD_RECKONING = 4   # Position is calculated by GPS and combined with dead reckoning. 
uint16 GPS_FIX_STATE_TIME_ONLY_FIX = 5             # Only time is fixed. 
float32 fix_state   

float32 vertical_accuracy                           # Vertical position accuracy (mm), the smaller, the better.
float32 horizontal_accuracy                         # Horizontal position accuracy (mm), the smaller, the better.
float32 speed_accuracy                              # Speed accuracy (cm/s), the smaller, the better.
uint32 num_gps_satellites_used                      # Number of GPS satellites used for fixing position.
uint32 num_glonass_satellites_used                  # Number of GLONASS satellites used for fixing position.
uint16 num_total_satellites_used                    # Total number of satellites used for fixing position.
uint16 gps_counter                                  # Accumulated times of sending GPS data.