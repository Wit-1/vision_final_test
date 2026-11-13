# Source image time/frame; predicted pose is for prediction_stamp in the same assumed-stable camera frame.
std_msgs/Header header
time prediction_stamp
float64 prediction_time_s
uint32 target_id
bool valid
bool geometry_ready
uint8 plate_count
int32 current_slot
int32 predicted_slot
int32 detection_index
bool is_large
float64 width_m
float64 height_m
geometry_msgs/Pose current
geometry_msgs/Pose predicted
geometry_msgs/Point center
geometry_msgs/Vector3 velocity
float64 phase
float64 angular_velocity
float64 radius_m
float64 delta_radius_m
# Odd-pair height minus even-pair height. Positive is upward, opposite camera Y.
float64 delta_height_m
geometry_msgs/Vector3 geometry_stddev
