#include "../include/datalogging/logging.h"
        
// sub callbacks
void datalogging::logging::brim_callback(std::shared_ptr<rclcpp::SerializedMessage> message) {
    // write data to file
    if ((logs_open) && (brim_running)) {
        rclcpp::Time time_stamp = this->now();
        writer_->write(message, "brim_data", "/GC/out/brim", time_stamp);
        //RCLCPP_INFO(this->get_logger(), "brim logging callback");
        //(log_file) << msg->header.stamp.sec << "," << msg->header.stamp.nanosec << "," << msg->desired_drone_position.x << "," << msg->desired_drone_position.y << "," << msg->desired_drone_position.z << "," << msg->phin_list.x << "," << msg->phin_list.y << "," << msg->phin_list.z << "," << msg->phin_dot_list.x << "," << msg->phin_dot_list.y << "," << msg->phin_dot_list.z << "," << msg->brim_freq << "," << msg->brim_count << "," << msg->brim_time << "\n";
    }
}

void datalogging::logging::bmn_callback(std::shared_ptr<rclcpp::SerializedMessage> message) {
    // write data to file
    if ((logs_open) && (bmn_running)) {
        rclcpp::Time time_stamp = this->now();
        writer_->write(message, "bmn_data", "/GC/out/bmn", time_stamp);
        //RCLCPP_INFO(this->get_logger(), "bmn logging callback");
        //(log_file) << msg->header.stamp.sec << "," << msg->header.stamp.nanosec << "," << msg->desired_drone_position.x << "," << msg->desired_drone_position.y << "," << msg->desired_drone_position.z << "," << msg->interface_force_list.x << "," << msg->interface_force_list.y << "," << msg->interface_force_list.z << "," << msg->bmn_freq << "\n";
    }
}

void datalogging::logging::rbquadsim_callback(std::shared_ptr<rclcpp::SerializedMessage> message) {
    // write data to file
    if ((logs_open) && (sim_running)) {
        rclcpp::Time time_stamp = this->now();
        writer_->write(message, "rbsim_data", "/GC/out/rbquadsim", time_stamp);
        //RCLCPP_INFO(this->get_logger(), "rbsim logging callback");
        //(log_file) << msg->header.stamp.sec << "," << msg->header.stamp.nanosec << "," << msg->position.x << "," << msg->position.y << "," << msg->position.z << "," << msg->velocity.x << "," << msg->velocity.y << "," << msg->velocity.z << "," << msg->acceleration.x << "," << msg->acceleration.y << "," << msg->acceleration.z << "," << msg->drag.x << "," << msg->drag.y << "," << msg->drag.z << "," << msg->gravity.x << "," << msg->gravity.y << "," << msg->gravity.z << "," << msg->interaction.x << "," << msg->interaction.y << "," << msg->interaction.z << "," << msg->orientation.w << "," <<  msg->orientation.x << "," << msg->orientation.y << "," << msg->orientation.z << "," <<  msg->angular_velocity.x << "," << msg->angular_velocity.y << "," << msg->angular_velocity.z << "," << msg->contact << "," << msg->pre_contact << "," << msg->post_contact << "," << msg->no_contact << "," << msg->sim_freq << "\n";
    }
}

void datalogging::logging::rrim_callback(std::shared_ptr<rclcpp::SerializedMessage> message) {
    // write data to file
    if ((logs_open) && (rrim_running)) {
        rclcpp::Time time_stamp = this->now();
        writer_->write(message, "prim_data", "/GC/out/prim", time_stamp);
        //RCLCPP_INFO(this->get_logger(), "prim logging callback");
        //(log_file) << msg->header.stamp.sec << "," << msg->header.stamp.nanosec << "," << msg->actual_drone_position.x << "," << msg->actual_drone_position.y << "," << msg->actual_drone_position.z << "," << msg->phin_list.x << "," << msg->phin_list.y << "," << msg->phin_list.z << "," << msg->phin_dot_list.x << "," << msg->phin_dot_list.y << "," << msg->phin_dot_list.z << "," << msg->rrim_freq << "," << msg->rrim_count << "," << msg->rrim_time << "\n";
    }
}

void datalogging::logging::rpi_callback(std::shared_ptr<rclcpp::SerializedMessage> message) {
    // write data to file
    if ((logs_open) && (rpicomms_running)) {
        rclcpp::Time time_stamp = this->now();
        writer_->write(message, "rpi_data", "/GC/out/rpicomms", time_stamp);
        //RCLCPP_INFO(this->get_logger(), "rpi logging callback");
        //(log_file) << msg->header.stamp.sec << "," << msg->header.stamp.nanosec << "," << msg->actual_drone_position.x << "," << msg->actual_drone_position.y << "," << msg->actual_drone_position.z << "," << msg->actual_drone_orientation.w << "," << msg->actual_drone_orientation.x << "," << msg->actual_drone_orientation.y << "," << msg->actual_drone_orientation.z << "," << msg->actual_drone_velocity.x << "," << msg->actual_drone_velocity.y << "," << msg->actual_drone_velocity.z << "," << msg->actual_drone_acceleration.x << "," << msg->actual_drone_acceleration.y << "," << msg->actual_drone_acceleration.z << "," << msg->rpi_freq << "\n";
    }
}

// log setup callback
void datalogging::logging::logsetup_callback(std::shared_ptr<rclcpp::SerializedMessage> message) {
    rclcpp::Time time_stamp = this->now();
    writer_->write(message, "log_data", "/GC/internal/logsetup", time_stamp);
}

void datalogging::logging::publish_log_state() {
    commsmsgs::msg::Logctrlbools msg{};
    msg.header.stamp = this->now();
    msg.opened = logs_open;
    msg.closed = logs_closed;
    msg.cleared = logs_cleared;
    //RCLCPP_INFO(this->get_logger(), "Publishing log state opened: %d closed: %d cleared: %d", logs_open, logs_closed, logs_cleared);
    logs_publisher_->publish(msg);
}

// opens all the log files once the info has been published from the gui node
void datalogging::logging::ctrl_logs_callback(const commsmsgs::msg::Guicontrols::UniquePtr & msg) {
    bool open_cmd = msg->open_logs;
    bool close_cmd = msg->close_logs;
    bool clear_cmd = msg->clear_logs;

    sim_running = msg->start_simulation;
    brim_running = msg->start_bmn;
    bmn_running = msg->start_bmn;
    rpicomms_running = msg->start_rpicomms;
    rrim_running = msg->start_rpicomms;

    if ((open_cmd) & (logs_closed)) {
        logs_open = true;
        logs_closed = false;
    }
    if ((close_cmd) & (logs_open)) {
        logs_closed = true;
        logs_open = false;
    }
}

