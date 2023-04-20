uint8 payload_index 1 # see enum E_DjiMountPosition in dji_typedef.h
uint8 rotation_mode # see E_DjiGimbalRotationMode in dji_gimbal_manager.h 0 /*!< Incremental angle rotation mode, represents rotating gimbal specified angles based on current angles. */
                                                                        # 1, /*!< Absolute angle rotation mode, represents rotating gimbal to specified angles in the ground coordinate. */
                                                                        # 2  /*!< Speed rotation mode, specifies rotation speed of gimbal in the ground coordinate. */
float32 pitch # unit: radians
float32 roll  # unit: radians
float32 yaw   # unit: radians
float32 time  # /*!< Expect execution time of gimbal rotation, unit: second. */
