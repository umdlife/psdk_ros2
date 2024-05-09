
uint8 DJI_CAMERA_FILE_TYPE_JPEG = 0
uint8 DJI_CAMERA_FILE_TYPE_DNG = 1 
uint8 DJI_CAMERA_FILE_TYPE_MOV = 2
uint8 DJI_CAMERA_FILE_TYPE_MP4 = 3
uint8 DJI_CAMERA_FILE_TYPE_TIFF = 5
uint8 DJI_CAMERA_FILE_TYPE_PCD = 24
uint8 DJI_CAMERA_FILE_TYPE_UNKNOWN = 255

std_msgs/Header header
uint8  file_type
uint32 file_size
uint32 file_index
string file_name
