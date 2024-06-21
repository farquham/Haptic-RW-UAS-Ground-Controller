#include "../include/rpicomms/rpicomms.h"

// /**
//  * @brief Publish a trajectory setpoint from data pipe struct
//  */
void RPI::rpicomms::publish_trajectory_setpoint()
{
	geometry_msgs::msg::Point msg{};
	msg.x = drone_position_cmd[0];
	msg.y = drone_position_cmd[1];
	msg.z = drone_position_cmd[2];
	//this->get_logger()->info("Publishing setpoint: x: " + std::to_string(msg.x) + " y: " + std::to_string(msg.y) + " z: " + std::to_string(msg.z));
	setpoint_publisher_->publish(msg);
}

// subscriber to recieve the drone position, velocity and acceleration from the rpi companion comp
void RPI::rpicomms::vehicle_pose_callback(const geometry_msgs::msg::PoseStamped::UniquePtr & msg)
{
	drone_position_actual[0] = msg->pose.position.x;
	drone_position_actual[1] = msg->pose.position.y;
	drone_position_actual[2] = msg->pose.position.z;
	drone_orientation_actual[0] = msg->pose.orientation.x;
	drone_orientation_actual[1] = msg->pose.orientation.y;
	drone_orientation_actual[2] = msg->pose.orientation.z;
	drone_orientation_actual[3] = msg->pose.orientation.w;
	RCLCPP_INFO(this->get_logger(), "Recieved drone position: x: %f y: %f z: %f", drone_position_actual[0], drone_position_actual[1], drone_position_actual[2]);
	RCLCPP_INFO(this->get_logger(), "Recieved drone orientation: x: %f y: %f z: %f w: %f", drone_orientation_actual[0], drone_orientation_actual[1], drone_orientation_actual[2], drone_orientation_actual[3]);
}

void RPI::rpicomms::vehicle_twist_callback(const geometry_msgs::msg::TwistStamped::UniquePtr & msg)
{
	drone_velocity_actual[0] = msg->twist.angular.x;
	drone_velocity_actual[1] = msg->twist.angular.y;
	drone_velocity_actual[2] = msg->twist.angular.z;
	RCLCPP_INFO(this->get_logger(), "Recieved drone angular velocity: x: %f y: %f z: %f", drone_velocity_actual[0], drone_velocity_actual[1], drone_velocity_actual[2]);
}

void RPI::rpicomms::vehicle_accel_callback(const geometry_msgs::msg::AccelStamped::UniquePtr & msg)
{
	drone_acceleration_actual[0] = msg->accel.linear.x;
	drone_acceleration_actual[1] = msg->accel.linear.y;
	drone_acceleration_actual[2] = msg->accel.linear.z;
	RCLCPP_INFO(this->get_logger(), "Recieved drone linear acceleration: x: %f y: %f z: %f", drone_acceleration_actual[0], drone_acceleration_actual[1], drone_acceleration_actual[2]);
}

// subscriber to recieve the ddc from the bmn comp
void RPI::rpicomms::prim_callback(const commsmsgs::msg::Rrimpub::UniquePtr & msg)
{
	drone_position_cmd[0] = msg->rbsim_position.x;
	drone_position_cmd[1] = msg->rbsim_position.y;
	drone_position_cmd[2] = msg->rbsim_position.z;
	//RCLCPP_INFO(this->get_logger(), "Recieved drone position command: x: %f y: %f z: %f", drone_position_cmd[0], drone_position_cmd[1], drone_position_cmd[2]);
}

// subscriber to recieve the gui controls from the gui
void RPI::rpicomms::guicontrols_callback(const commsmsgs::msg::Guicontrols::UniquePtr & msg)
{
    if ((msg->start_rpicomms) && (!(msg->stop_rpicomms))){
        run = true;
    }
    else if ((msg->stop_rpicomms) && (!(msg->start_rpicomms))){
        run = false;
    }
}

// publisher to send the drone position, velocity and acceleration to the rest of the system
void RPI::rpicomms::publish_vehicle_state()
{
	commsmsgs::msg::Rpicommspub msg{};
	msg.header.stamp = this->now();
	msg.running = run;
	msg.actual_drone_position.x = drone_position_actual[0];
	msg.actual_drone_position.y = drone_position_actual[1];
	msg.actual_drone_position.z = drone_position_actual[2];
	msg.actual_drone_orientation.x = drone_orientation_actual[0];
	msg.actual_drone_orientation.y = drone_orientation_actual[1];
	msg.actual_drone_orientation.z = drone_orientation_actual[2];
	msg.actual_drone_orientation.w = drone_orientation_actual[3];
	msg.actual_drone_velocity.x = drone_velocity_actual[0];
	msg.actual_drone_velocity.y = drone_velocity_actual[1];
	msg.actual_drone_velocity.z = drone_velocity_actual[2];
	msg.actual_drone_acceleration.x = drone_acceleration_actual[0];
	msg.actual_drone_acceleration.y = drone_acceleration_actual[1];
	msg.actual_drone_acceleration.z = drone_acceleration_actual[2];
	rpicomms_publisher_->publish(msg);
}