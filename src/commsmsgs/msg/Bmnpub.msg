# msg for all data the bubble method nav has to publish or output
# to the rest of the system

# timestamp
std_msgs/Header header

# running feedback bool
bool running

# position data
# subbed to by brim node
geometry_msgs/Vector3 interface_force_list
# subbed to by brim node
geometry_msgs/Point desired_drone_position

# frequency data
# subbed to by logging node
float64 bmn_freq