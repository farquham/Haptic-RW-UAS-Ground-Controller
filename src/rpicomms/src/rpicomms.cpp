#include "PX4Comms.h"

// /**
//  * @brief Publish a trajectory setpoint from data pipe struct
//  */
void RPI::rpicomms::publish_trajectory_setpoint()
{
	Eigen::Vector3d sp;
	lock->lock();
	sp = ptr->B_DDC;
	lock->unlock();
	Eigen::Vector3f spf = sp.cast<float>();
	geometry_msgs::msg::point msg{};
	float zp = -1*(spf[2]);
	msg.x = spf[1];
	msg.y = spf[0];
	msg.z = zp;
	setpoint_publisher_->publish(msg);
}