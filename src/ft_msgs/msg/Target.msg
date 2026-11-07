# Input image time and optical frame. All positions are in metres.
std_msgs/Header header
# True when a vehicle observation was matched and a future facing plate was found.
bool valid
# Vehicle trajectory ID. Zero means no active vehicle.
uint32 target_id
# Estimated plate position at the input image time.
geometry_msgs/Point position
# Predicted visible plate at image time + T, possibly a different plate.
geometry_msgs/Point position_predict
