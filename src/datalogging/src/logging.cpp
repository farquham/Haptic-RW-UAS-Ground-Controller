#include "../include/datalogging/logging.h"
        
// sub callbacks
void datalogging::logging::brim_callback(const commsmsgs::msg::Brimpub::UniquePtr & msg, std::ofstream & log_file) {
    // write data to file
    if (logs_open) {
        (log_file) << msg->header.stamp.sec << "," << msg->header.stamp.nanosec << "," << msg->desired_drone_position.x << "," << msg->desired_drone_position.y << "," << msg->desired_drone_position.z << "," << msg->phin_list.x << "," << msg->phin_list.y << "," << msg->phin_list.z << "," << msg->phin_dot_list.x << "," << msg->phin_dot_list.y << "," << msg->phin_dot_list.z << "," << msg->brim_freq << "," << msg->brim_count << "," << msg->brim_time << "\n";
    }
}

void datalogging::logging::bmn_callback(const commsmsgs::msg::Bmnpub::UniquePtr & msg, std::ofstream & log_file) {
    // write data to file
    if (logs_open) {
        (log_file) << msg->header.stamp.sec << "," << msg->header.stamp.nanosec << "," << msg->desired_drone_position.x << "," << msg->desired_drone_position.y << "," << msg->desired_drone_position.z << "," << msg->interface_force_list.x << "," << msg->interface_force_list.y << "," << msg->interface_force_list.z << "," << msg->bmn_freq << "\n";
    }
}

void datalogging::logging::rbquadsim_callback(const commsmsgs::msg::Rbquadsimpub::UniquePtr & msg, std::ofstream & log_file) {
    // write data to file
    if (logs_open) {
        (log_file) << msg->header.stamp.sec << "," << msg->header.stamp.nanosec << "," << msg->position.x << "," << msg->position.y << "," << msg->position.z << "," << msg->velocity.x << "," << msg->velocity.y << "," << msg->velocity.z << "," << msg->acceleration.x << "," << msg->acceleration.y << "," << msg->acceleration.z << "," << msg->drag.x << "," << msg->drag.y << "," << msg->drag.z << "," << msg->gravity.x << "," << msg->gravity.y << "," << msg->gravity.z << "," << msg->interaction.x << "," << msg->interaction.y << "," << msg->interaction.z << "," << msg->orientation.w << "," <<  msg->orientation.x << "," << msg->orientation.y << "," << msg->orientation.z << "," <<  msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z << "," << msg->contact << "," << msg->pre_contact << "," << msg->post_contact << "," << msg->no_contact << "," << msg->sim_freq << "\n";
    }
}

void datalogging::logging::rrim_callback(const commsmsgs::msg::Rrimpub::UniquePtr & msg, std::ofstream & log_file) {
    // write data to file
    if (logs_open) {
        (log_file) << msg->header.stamp.sec << "," << msg->header.stamp.nanosec << "," << msg->actual_drone_position.x << "," << msg->actual_drone_position.y << "," << msg->actual_drone_position.z << "," << msg->phin_list.x << "," << msg->phin_list.y << "," << msg->phin_list.z << "," << msg->phin_dot_list.x << "," << msg->phin_dot_list.y << "," << msg->phin_dot_list.z << "," << msg->rrim_freq << "," << msg->rrim_count << "," << msg->rrim_time << "\n";
    }
}

void datalogging::logging::rpi_callback(const commsmsgs::msg::Rpicommspub::UniquePtr & msg, std::ofstream & log_file) {
    // write data to file
    if (logs_open) {
        (log_file) << msg->header.stamp.sec << "," << msg->header.stamp.nanosec << "," << msg->actual_drone_position.x << "," << msg->actual_drone_position.y << "," << msg->actual_drone_position.z << "," << msg->actual_drone_orientation.w << "," << msg->actual_drone_orientation.x << "," << msg->actual_drone_orientation.y << "," << msg->actual_drone_orientation.z << "," << msg->actual_drone_velocity.x << "," << msg->actual_drone_velocity.y << "," << msg->actual_drone_velocity.z << "," << msg->actual_drone_acceleration.x << "," << msg->actual_drone_acceleration.y << "," << msg->actual_drone_acceleration.z << "," << msg->rpi_freq << "\n";
    }
}

void datalogging::logging::publish_log_state() {
    commsmsgs::msg::Logctrlbools msg{};
    msg.header.stamp = this->now();
    msg.opened = logs_open;
    msg.closed = logs_closed;
    msg.cleared = logs_cleared;
    logs_publisher_->publish(msg);
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
    log_file->open(file_name, std::ios::out | std::ios::app);
}

// closes a log file
void datalogging::logging::close_log_file(std::ofstream & log_file) {
    log_file->close();
}

// closes then opens log file in truncate mode to clear data then closes again then opens in append mode
void datalogging::logging::clear_log_file(std::string file_name, std::ofstream & log_file) {
    log_file->close();
    log_file->open(file_name, std::ios::out | std::ios::trunc);
    log_file->close();
    log_file->open(file_name, std::ios::out | std::ios::app);
}

// names all the log files once the info has been published from the gui node
void datalogging::logging::name_logs_callback(const commsmsgs::msg::Logsetup::UniquePtr & msg) {
    name_log_file(brim_file, msg->participant_id, msg->experiment_id, msg->movement_type, msg->rim_type, "brim");
    name_log_file(bmn_file, msg->participant_id, msg->experiment_id, msg->movement_type, msg->rim_type, "bmn");
    name_log_file(rbquadsim_file, msg->participant_id, msg->experiment_id, msg->movement_type, msg->rim_type, "rbquadsim");
    name_log_file(rrim_file, msg->participant_id, msg->experiment_id, msg->movement_type, msg->rim_type, "rrim");
    name_log_file(rpi_file, msg->participant_id, msg->experiment_id, msg->movement_type, msg->rim_type, "rpi");
}

// opens all the log files once the info has been published from the gui node
void datalogging::logging::ctrl_logs_callback(const commsmsgs::msg::Guicontrols::UniquePtr & msg) {
    bool open_cmd = msg->open_logs;
    bool close_cmd = msg->close_logs;
    bool clear_cmd = msg->clear_logs;

    if ((open_cmd) & (logs_closed)) {
        logs_open = true;
        logs_closed = false;
        open_log_file(&brim_file, &brim_log_file);
        open_log_file(&bmn_file, &bmn_log_file);
        open_log_file(&rbquadsim_file, &rbquadsim_log_file);
        open_log_file(&rrim_file, &rrim_log_file);
        open_log_file(&rpi_file, &rpi_log_file);
    }
    if ((close_cmd) & (logs_open)) {
        logs_closed = true;
        logs_open = false;
        close_log_file(&brim_log_file);
        close_log_file(&bmn_log_file);
        close_log_file(&rbquadsim_log_file);
        close_log_file(&rrim_log_file);
        close_log_file(&rpi_log_file);
    }
    if ((clear_cmd) & (logs_open)) {
        logs_cleared = true;
        logs_open = false;
        clear_log_file(&brim_file, &brim_log_file);
        clear_log_file(&bmn_file, &bmn_log_file);
        clear_log_file(&rbquadsim_file, &rbquadsim_log_file);
        clear_log_file(&rrim_file, &rrim_log_file);
        clear_log_file(&rpi_file, &rpi_log_file);
        logs_cleared = false;
        logs_open = true;
    }
}

