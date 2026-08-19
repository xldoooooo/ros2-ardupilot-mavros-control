# 视频截图命令；纯视频手动命令可由地面站直发，航点命令由机载控制节点发布。
uint8 KIND_MANUAL=0
uint8 KIND_GCS_WAYPOINT=1
uint8 KIND_UPSTREAM_WAYPOINT=2

std_msgs/Header header
string source_id
uint64 sequence
uint8 kind
uint64 mission_sequence
uint32 waypoint_index
string photo_no
