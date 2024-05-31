# msg for all data the bubble method nav rim has to publish or output
# to the rest of the system

# timestamp
std_msgs/Header header

# running feedback bool
bool running

# position data
# subbed by bmn node
geometry_msgs/Vector3 phin_list
geometry_msgs/Vector3 phin_dot_list
# subbed by rbsim node
geometry_msgs/Point desired_drone_position

# frequency data of loops
# subbed by logging node
float64 brim_freq
float64 brim_count
float64 brim_time
