# 机载端对一条高层命令发布的可靠执行结果或进度。
uint8 STATUS_RUNNING=0
uint8 STATUS_SUCCEEDED=1
uint8 STATUS_FAILED=2
uint8 STATUS_CANCELLED=3
uint8 STATUS_REJECTED=4

std_msgs/Header header
string source_id
uint64 sequence
string command
uint8 status
bool final
string message
uint32 waypoint_index
uint32 waypoint_count
