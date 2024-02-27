#include <../include/datalogging/logging.h>
        
// sub callbacks
void datalogging::logging::brim_callback(const commsmsgs::msg::brimPub::UniquePtr & msg, std::ofstream & log_file) {
    // write data to file
    log_file << msg->header->stamp->sec << "," << msg->header->stamp->nanosec << "," << msg->desired_drone_position->x << "," << msg->desired_drone_position->y << "," << msg->desired_drone_position->z << "," << msg->phin_list->x << "," << msg->phin_list->y << "," << msg->phin_list->z << "," << msg->phin_dot_list->x << "," << msg->phin_dot_list->y << "," << msg->phin_dot_list->z << "," << msg->brim_freq << "," << msg->brim_count << "," << msg->brim_time << "\n";
}

void datalogging::logging::bmn_callback(const commsmsgs::msg::bmnPub::UniquePtr & msg) {
    // write data to file
    log_file << msg->header->stamp->sec << "," << msg->header->stamp->nanosec << "," << msg->desired_drone_position->x << "," << msg->desired_drone_position->y << "," << msg->desired_drone_position->z << "," << msg->interface_force_list->x << "," << msg->interface_force_list->y << "," << msg->interface_force_list->z << "," << msg->bmn_freq << "\n";
}

void datalogging::logging::rbquadsim_callback(const commsmsgs::msg::rbquadsimPub::UniquePtr & msg) {
    // write data to file
    log_file << msg->header->stamp->sec << "," << msg->header->stamp->nanosec << "," << msg->position->x << "," << msg->position->y << "," << msg->position->z << "," << msg->velocity->x << "," << msg->velocity->y << "," << msg->velocity->z << "," << msg->acceleration->x << "," << msg->acceleration->y << "," << msg->acceleration->z << "," << msg->drag->x << "," << msg->drag->y << "," << msg->drag->z << "," << msg->gravity->x << "," << msg->gravity->y << "," << msg->gravity->z << "," << msg->interaction->x << "," << msg->interaction->y << "," << msg->interaction->z << "," << msg->contact << "," << msg->pre_contact << "," << msg->post_contact << "," << msg->no_contact << "," << msg->sim_freq << "\n";
}

void datalogging::logging::rrim_callback(const commsmsgs::msg::rrimPub::UniquePtr & msg) {
    // write data to file
    log_file << msg->header->stamp->sec << "," << msg->header->stamp->nanosec << "," << msg->actual_drone_position->x << "," << msg->actual_drone_position->y << "," << msg->actual_drone_position->z << "," << msg->phin_list->x << "," << msg->phin_list->y << "," << msg->phin_list->z << "," << msg->phin_dot_list->x << "," << msg->phin_dot_list->y << "," << msg->phin_dot_list->z << "," << msg-rrim_freq << "," << msg->rrim_count << "," << msg->rrim_time << "\n";
}

void datalogging::logging::rpi_callback(const commsmsgs::msg::rpicommsPub::UniquePtr & msg) {
    // write data to file
    log_file << msg->header->stamp->sec << "," << msg->header->stamp->nanosec << "," << msg->actual_drone_position->x << "," << msg->actual_drone_position->y << "," << msg->actual_drone_position->z << "," << msg->actual_drone_velocity->x << "," << msg->actual_drone_velocity->y << "," << msg->actual_drone_velocity->z << "," << msg->actual_drone_acceleration->x << "," << msg->actual_drone_acceleration->y << "," << msg->actual_drone_acceleration->z << "," << msg->rpi_freq << "\n";
}

// logging files helpers
// makes the correct logging file name
void datalogging::logging::name_log_file(std::string & file_name, int P_ID, int e_num, std::string type, int r_type, std::string file_type) {
    std::string Part_ID = std::to_string(P_ID);
    std::string exp_num = std::to_string(e_num);
    std::string rim_type = std::to_string(r_type);
    file_name = "/home/farquham/Desktop/Experimental_Data/Multi User Study/" + Part_ID + "/EXP_" + exp_num + "/log_E_" + exp_num + "_M_" + type + "_R_" + rim_type + "_" + file_type + ".csv";
}

// opens a log file in append mode to add data
void datalogging::logging::open_log_file(std::string file_name, std::ofstream & log_file) {
    log_file.open(file_name, std::ios::out | std::ios::app);
}

// closes a log file
void datalogging::logging::close_log_file(std::ofstream & log_file) {
    log_file.close();
}

// closes then opens log file in truncate mode to clear data then closes again then opens in append mode
void datalogging::logging::clear_log_file(std::string file_name, std::ofstream & log_file) {
    log_file.close();
    log_file.open(file_name, std::ios::out | std::ios::trunc);
    log_file.close();
    log_file.open(file_name, std::ios::out | std::ios::app);
}

// names all the log files once the info has been published from the gui node
void name_logs_callback(const commsmsgs::msg::Logsetup::UniquePtr & msg, std::string & brim_file, std::string & bmn_file, std::string & rbquadsim_file, std::string & rrim_file, std::string & rpi_file) {
    name_log_file(brim_file, msg->participant_id, msg->experiment_id, msg->movement_type, msg->rim_type, msg->file_type1);
    name_log_file(bmn_file, msg->participant_id, msg->experiment_id, msg->movement_type, msg->rim_type, msg->file_type2);
    name_log_file(rbquadsim_file, msg->participant_id, msg->experiment_id, msg->movement_type, msg->rim_type, msg->file_type3);
    name_log_file(rrim_file, msg->participant_id, msg->experiment_id, msg->movement_type, msg->rim_type, msg->file_type4);
    name_log_file(rpi_file, msg->participant_id, msg->experiment_id, msg->movement_type, msg->rim_type, msg->file_type5);
}

// opens all the log files once the info has been published from the gui node
void ctrl_logs_callback(const commsmsgs::msg::Logctrlbools::UniquePtr & msg, std::string & brim_file, std::ofstream & brim_log_file, std::string & bmn_file, std::ofstream & bmn_log_file, std::string & rbquadsim_file, std::ofstream & rbquadsim_log_file, std::string & rrim_file, std::ofstream & rrim_log_file, std::string & rpi_file, std::ofstream & rpi_log_file) {
    bool open = msg->open;
    bool close = msg->close;
    bool clear = msg->clear;

    if (open) {
        open_log_file(brim_file, brim_log_file);
        open_log_file(bmn_file, bmn_log_file);
        open_log_file(rbquadsim_file, rbquadsim_log_file);
        open_log_file(rrim_file, rrim_log_file);
        open_log_file(rpi_file, rpi_log_file);
    } else if (close) {
        close_log_file(brim_log_file);
        close_log_file(bmn_log_file);
        close_log_file(rbquadsim_log_file);
        close_log_file(rrim_log_file);
        close_log_file(rpi_log_file);
    } else if (clear) {
        clear_log_file(brim_file, brim_log_file);
        clear_log_file(bmn_file, bmn_log_file);
        clear_log_file(rbquadsim_file, rbquadsim_log_file);
        clear_log_file(rrim_file, rrim_log_file);
        clear_log_file(rpi_file, rpi_log_file);
    }
}

