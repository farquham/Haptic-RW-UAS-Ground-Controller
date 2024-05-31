# msg to control what the logging node does

# timestamp
std_msgs/Header header

# open, close, clear logs
bool open_logs
bool close_logs
bool clear_logs

# start, stop, and reset the simulation loop
bool start_simulation
bool stop_simulation
bool reset_simulation

# start, stop, and reset the bmn and brim loops
bool start_bmn
bool stop_bmn
bool reset_bmn

# start, stop, and reset the rpicomms and prim loops
bool start_rpicomms
bool stop_rpicomms
bool reset_rpicomms

# irl drone commands
bool takeoff
bool land