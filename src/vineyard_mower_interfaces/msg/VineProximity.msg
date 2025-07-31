# VineProximity.msg
# Message for reporting vine proximity information from detection systems

# Header for timestamp and frame information
std_msgs/Header header

# Distance to closest detected vine (meters)
# -1.0 indicates no vines detected
float32 closest_distance

# Safety status based on proximity
bool safety_alert          # True if within critical safety distance
bool warning_active        # True if within warning distance

# Number of vines detected in current frame
uint32 vine_count

# Confidence of detection (0.0 to 1.0)
float32 detection_confidence

# Position of closest vine in camera frame
geometry_msgs/Point closest_vine_position

# Array of all detected vine positions
geometry_msgs/Point[] vine_positions
