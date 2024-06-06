# msg for all data the main quadcopter/rigidbody sim has to publish or output
# to the rest of the system

# timestamp
std_msgs/Header header

# running feedback bool
bool running

# drone state data
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration

# drone state data other
geometry_msgs/Vector3 drag
geometry_msgs/Vector3 gravity
geometry_msgs/Vector3 interaction
geometry_msgs/Quaternion orientation 
geometry_msgs/Vector3 angular_velocity
Matrixthreef rotation_matrix

# sparse matrices
std_msgs/Float64MultiArray m_inv
std_msgs/Float64MultiArray ac
std_msgs/Float64MultiArray vg

# contact state data
bool contact
bool pre_contact
bool post_contact
bool no_contact

# frequency data
float64 sim_freq