# msg for all data the bubble method nav rim has to publish or output
# to the rest of the system

# timestamp
std_msgs/Header header

# running feedback bool
bool running

# irl drone state data
bool drone_is_flying
bool drone_is_landed

# position data
# subbed by rbsim node
geometry_msgs/Point actual_drone_position
geometry_msgs/Quaternion actual_drone_orientation
geometry_msgs/Point actual_drone_velocity
geometry_msgs/Point actual_drone_acceleration

# frequency data of loops
# subbed by logging node
float64 rpi_freq