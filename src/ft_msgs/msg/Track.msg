# Input image time and camera frame. Position is in metres; velocity is in m/s.
std_msgs/Header header
# 0: no active track; positive IDs survive short missed observations.
uint32 id
# true only when this frame initializes or corrects the track with an observation.
bool tracking
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Point position_predict
