# Goal lifecycle and line checks in map; zero ID is a capability heartbeat.
uint8 PLAN=1
uint8 CHECK_LINE=2
uint8 VALIDATE_TRAJECTORY=3
uint8 CANCEL=4
std_msgs/Header header
string controller_session
uint64 task_revision
# 航点序号从 1 开始；会话/任务/航点/请求身份必须完整回显。
uint32 waypoint_index
uint64 request_id
uint8 mode
geometry_msgs/Point start
geometry_msgs/Vector3 velocity
geometry_msgs/Point goal
guided_interfaces/PolynomialSegment[] segments
float64 trajectory_elapsed
