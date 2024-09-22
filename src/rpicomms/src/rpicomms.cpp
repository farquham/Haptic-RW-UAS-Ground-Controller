#include "../include/rpicomms/rpicomms.h"

// /**
//  * @brief Publish a trajectory setpoint from data pipe struct
//  */
void RPI::rpicomms::publish_trajectory_setpoint()
{
	// if the drone position has yet to be relayed do nothing
	if ((drone_position_actual[0] == 0.0) && (drone_position_actual[1] == 0.0) && (drone_position_actual[2] == 0.0)) {
		return;
	// if the drone position command is set to takeoff then set the setpoint to the current position plus 1m in the z direction
	} else if ((drone_position_cmd[0] == 0.0) && (drone_position_cmd[1] == 0.0) && (drone_position_cmd[2] == 1.0)) {
		geometry_msgs::msg::Point msg{};
		msg.x = drone_position_cmd[0] + drone_position_actual[0];
		msg.y = drone_position_cmd[1] + drone_position_actual[1];
		msg.z = drone_position_cmd[2] + drone_position_actual[2];
		//this->get_logger()->info("Publishing setpoint: x: " + std::to_string(msg.x) + " y: " + std::to_string(msg.y) + " z: " + std::to_string(msg.z));
		setpoint_publisher_->publish(msg);
	// if the drone is attempting to land then send the land setpoint to the rpi
	} else if (this->time_to_land(drone_position_cmd, drone_position_actual)) {
		geometry_msgs::msg::Point msg{};
		msg.x = 1000.0;
		msg.y = 1000.0;
		msg.z = 1000.0;
		//this->get_logger()->info("Publishing setpoint: x: " + std::to_string(msg.x) + " y: " + std::to_string(msg.y) + " z: " + std::to_string(msg.z));
		setpoint_publisher_->publish(msg);
	// otherwise behave normally and send the setpoint to the rpi
	} else {
		geometry_msgs::msg::Point msg{};
		msg.x = drone_position_cmd[0];
		msg.y = drone_position_cmd[1];
		msg.z = drone_position_cmd[2];
		//this->get_logger()->info("Publishing setpoint: x: " + std::to_string(msg.x) + " y: " + std::to_string(msg.y) + " z: " + std::to_string(msg.z));
		setpoint_publisher_->publish(msg);
	}
}

bool RPI::rpicomms::time_to_land(Eigen::Vector3d cmd_pos, Eigen::Vector3d current_pos) {
	Eigen::Vector3d zeros = {0.0, 0.0, 0.25};
	Eigen::Vector3d zero_diff = cmd_pos - zeros;
	Eigen::Vector3d current_diff = cmd_pos - current_pos;
	if (zero_diff.norm() < 0.15) {
		if (current_diff.norm() < 0.15) {
			land_counter++;
		} else {
			land_counter = 0;
		}
	} else {
		land_counter = 0;
	}
	if (land_counter > 100) {
		return true;
	} else {
		return false;
	}
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
	// RCLCPP_INFO(this->get_logger(), "Recieved drone position: x: %f y: %f z: %f", drone_position_actual[0], drone_position_actual[1], drone_position_actual[2]);
	// RCLCPP_INFO(this->get_logger(), "Recieved drone orientation: x: %f y: %f z: %f w: %f", drone_orientation_actual[0], drone_orientation_actual[1], drone_orientation_actual[2], drone_orientation_actual[3]);
}

void RPI::rpicomms::vehicle_twist_callback(const geometry_msgs::msg::TwistStamped::UniquePtr & msg)
{
	drone_velocity_actual[0] = msg->twist.angular.x;
	drone_velocity_actual[1] = msg->twist.angular.y;
	drone_velocity_actual[2] = msg->twist.angular.z;
	// RCLCPP_INFO(this->get_logger(), "Recieved drone angular velocity: x: %f y: %f z: %f", drone_velocity_actual[0], drone_velocity_actual[1], drone_velocity_actual[2]);
}

void RPI::rpicomms::vehicle_accel_callback(const geometry_msgs::msg::AccelStamped::UniquePtr & msg)
{
	drone_acceleration_actual[0] = msg->accel.linear.x;
	drone_acceleration_actual[1] = msg->accel.linear.y;
	drone_acceleration_actual[2] = msg->accel.linear.z;
	// RCLCPP_INFO(this->get_logger(), "Recieved drone linear acceleration: x: %f y: %f z: %f", drone_acceleration_actual[0], drone_acceleration_actual[1], drone_acceleration_actual[2]);
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