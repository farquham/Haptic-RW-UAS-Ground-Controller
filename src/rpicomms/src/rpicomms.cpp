#include "PX4Comms.h"

// /**
//  * @brief Publish a trajectory setpoint from data pipe struct
//  */
void RPI::rpicomms::publish_trajectory_setpoint()
{
	geometry_msgs::msg::point msg{};
	msg.x = drone_position_cmd[0];
	msg.y = drone_position_cmd[1];
	msg.z = drone_position_cmd[2];
	setpoint_publisher_->publish(msg);
}

// subscriber to recieve the drone position, velocity and acceleration from the rpi companion comp
void RPI::rpicomms::vehicle_state_callback(const geometry_msgs::msg::point::UniquePtr & msg)
{
	drone_position_actual[0] = msg->x;
	drone_position_actual[1] = msg->y;
	drone_position_actual[2] = msg->z;
}

// subscriber to recieve the ddc from the bmn comp
void RPI::rpicomms::brim_callback(const commsmsgs::msg::brimpub::UniquePtr & msg)
{
	drone_position_cmd[0] = msg->desired_drone_position->x;
	drone_position_cmd[1] = msg->desired_drone_position->y;
	drone_position_cmd[2] = msg->desired_drone_position->z;
}