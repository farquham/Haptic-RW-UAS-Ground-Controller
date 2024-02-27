# msg to setup the logging node

# timestamp
std_msgs/Header header

# participant id number
int32 participant_id

# experiment id number
int32 experiment_id

# movement type id (free movement = FM, guided movement = GM)
string movement_type

# rim type id (ZOH = 0, FOH = 1, RIM = 2, RIM+ = 3)
int32 rim_type

# file type id
string file_type1
string file_type2
string file_type3
string file_type4
string file_type5