# 当前控制权持有者用于续租并刷新地面站本地时间基准的低带宽心跳。
# header.stamp 是地面站本地发送时间，绝不与机载绝对日期直接比较。
std_msgs/Header header
string source_id
uint64 sequence
uint32 lease_duration_ms
