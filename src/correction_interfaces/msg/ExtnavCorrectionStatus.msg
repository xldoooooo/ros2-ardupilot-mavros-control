# extnav 当前 Odin session、实际生效修正及 identity 保底状态。
std_msgs/Header header
string interface_version
bool service_available
bool odin_available
string odin_session_id
bool correction_valid
uint64 revision
uint8 reset_counter
float64 correction_x_m
float64 correction_y_m
float64 correction_yaw_deg
string applied_job_id
float64 raw_age_s
uint64 raw_messages
uint64 corrected_messages
string last_event
string last_error
