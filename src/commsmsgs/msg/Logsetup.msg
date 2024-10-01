# msg to setup the logging node

# timestamp
std_msgs/Header header

# participant id number
int32 participant_id

# experiment id number
int32 experiment_id

# movement type id (Guided Movement Task = GMT, Contact Task = CT)
string movement_type

# rim type id (ZOH = 0, FOH = 1, RIM = 2, RIM+ = 3)
int32 rim_type