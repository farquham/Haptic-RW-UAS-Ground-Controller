# msg for all data the px4 rim has to publish or output
# to the rest of the system

# timestamp
std_msgs/Header header

# running feedback bool
bool running

# position data
# subbed by px4comms node
geometry_msgs/Vector3 phin_list
geometry_msgs/Vector3 phin_dot_list
# subbed by rbsim node
geometry_msgs/Point rbsim_position
geometry_msgs/Point actual_drone_position

# frequency data of loops
# subbed by logging node
float64 rrim_freq
float64 rrim_count
float64 rrim_time