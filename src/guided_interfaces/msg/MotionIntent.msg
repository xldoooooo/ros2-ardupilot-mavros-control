# 一次有序且有有效期的键盘速度增量意图；连续控制状态只保存在机载端。
std_msgs/Header header
string source_id
uint64 sequence
uint32 ttl_ms
geometry_msgs/Vector3 velocity_delta
float64 yaw_rate_delta
