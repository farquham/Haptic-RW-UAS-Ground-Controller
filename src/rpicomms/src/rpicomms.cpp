#include "PX4Comms.h"

// /**
//  * @brief Publish a trajectory setpoint from data pipe struct
//  */
void PX4C::OffboardControl::publish_trajectory_setpoint(BMN::Var_Transfer* ptr, std::mutex* lock)
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
	trajectory_setpoint_publisher_->publish(msg);
}