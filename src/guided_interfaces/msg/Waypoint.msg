# 任务系绝对 ENU 航点、独立行为标志及可选巡检照片编号。
geometry_msgs/Point position
float64 yaw
bool has_photo_no
string photo_no

# 普通点默认不抓拍；行为标志可组合，后续行为可独立追加。
bool is_inspection false
bool is_calibration false
int32 calibration_tag_id 0
bool continue_window false
