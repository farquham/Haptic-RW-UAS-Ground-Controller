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
void RPI::rpicomms::vehicle_pose_callback(const geometry_msgs::msg::pose_stamped::UniquePtr & msg)
{
	drone_position_actual[0] = msg->pose->position->x;
	drone_position_actual[1] = msg->pose->position->y;
	drone_position_actual[2] = msg->pose->position->z;
	drone_orientation_actual[0] = msg->pose->orientation->x;
	drone_orientation_actual[1] = msg->pose->orientation->y;
	drone_orientation_actual[2] = msg->pose->orientation->z;
	drone_orientation_actual[3] = msg->pose->orientation->w;
}

void RPI::rpicomms::vehicle_twist_callback(const geometry_msgs::msg::twist_stamped::UniquePtr & msg)
{
	drone_velocity_actual[0] = msg->twist->linear->x;
	drone_velocity_actual[1] = msg->twist->linear->y;
	drone_velocity_actual[2] = msg->twist->linear->z;
}

void RPI::rpicomms::vehicle_accel_callback(const geometry_msgs::msg::accel_stamped::UniquePtr & msg)
{
	drone_acceleration_actual[0] = msg->accel->linear->x;
	drone_acceleration_actual[1] = msg->accel->linear->y;
	drone_acceleration_actual[2] = msg->accel->linear->z;
}

// subscriber to recieve the ddc from the bmn comp
void RPI::rpicomms::brim_callback(const commsmsgs::msg::brimpub::UniquePtr & msg)
{
	drone_position_cmd[0] = msg->desired_drone_position->x;
	drone_position_cmd[1] = msg->desired_drone_position->y;
	drone_position_cmd[2] = msg->desired_drone_position->z;
}

// publisher to send the drone position, velocity and acceleration to the rest of the system
void RPI::rpicomms::publish_vehicle_state()
{
	commsmsgs::msg::rpicommspub msg{};
	msg.header.stamp = this->now();
	msg->actual_drone_position->x = drone_position_actual[0];
	msg->actual_drone_position->y = drone_position_actual[1];
	msg->actual_drone_position->z = drone_position_actual[2];
	msg->actual_drone_orientation->x = drone_orientation_actual[0];
	msg->actual_drone_orientation->y = drone_orientation_actual[1];
	msg->actual_drone_orientation->z = drone_orientation_actual[2];
	msg->actual_drone_orientation->w = drone_orientation_actual[3];
	msg->actual_drone_velocity->x = drone_velocity_actual[0];
	msg->actual_drone_velocity->y = drone_velocity_actual[1];
	msg->actual_drone_velocity->z = drone_velocity_actual[2];
	msg->actual_drone_acceleration->x = drone_acceleration_actual[0];
	msg->actual_drone_acceleration->y = drone_acceleration_actual[1];
	msg->actual_drone_acceleration->z = drone_acceleration_actual[2];
	rpicomms_publisher_->publish(msg);
}