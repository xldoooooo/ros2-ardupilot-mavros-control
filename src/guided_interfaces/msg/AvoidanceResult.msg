# Native planning stream and line-query replies; horizon segments are intermediate references.
uint8 REACH_HORIZON=1
uint8 REACH_END=2
uint8 NO_PATH=3
uint8 INVALID_INPUT=5
uint8 NO_MAP=6
uint8 TIMEOUT=7
uint8 NODE_LIMIT=8
uint8 WAITING_FOR_INPUT=9
uint8 STALE_INPUT=10
uint8 FRAME_MISMATCH=11
std_msgs/Header header
string controller_session
uint64 task_revision
uint32 waypoint_index
uint64 request_id
# Monotonic native stream sequence; zero for heartbeats or local goal-delivery failures.
uint64 plan_sequence
uint8 mode
string bridge_session
string planner_session
uint64 coordinate_revision
bool ready
# Resolved native limits: horizontal norm and absolute vertical component.
float64 max_velocity_xy
float64 max_velocity_z
float64 max_acceleration_xy
float64 max_acceleration_z
uint8 status
string detail
guided_interfaces/PolynomialSegment[] segments
