#ifndef PX4Comms_H
#define PX4Comms_H

// include eigen dense and sparse and alse the quadcopter classes
#include <iostream>
#include <../../deps/Eigen3/Eigen/Dense>
// #include <Eigen/Dense>
// #include <Eigen/Sparse>
#include <chrono>
#include <cstdio>
#include <mutex>

// px4 stuff needed for communication with IRL drone
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include "BubbleMethod.h"

// some nanespace stuff
using namespace std::chrono;
using namespace std::chrono_literals;

namespace PX4C {
	// class for offboard control which starts a ROS2 node
	class OffboardControl : public rclcpp::Node {
	public:
		OffboardControl(BMN::Var_Transfer* ptr, std::mutex* lock) : Node("offboard_control")
		{
			// publishers for picking offboard mode, sending trajectory setpoints and sending vehicle commands
			setpoint_publisher_ = this->create_publisher<geometry_msgs::msgs::point>("/rpi/in/ext_cmdPoint", 10);

			// state subscriber for recieving IRL drone position, velocity and acceleration
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
			vehicle_state_subscriber_ = this->create_subscription<geometry_msgs::msgs::point>("/rpi/out/localPoint", qos, [this, ptr, lock](const geometry::msgs::point::UniquePtr msg) {
				//subscribe_vehicle_state(ptr, lock, msg);
				Eigen::Vector3d actual_position;
				actual_position[0] = msg->y;
				actual_position[1] = msg->x;
				actual_position[2] = -msg->z;
				lock->lock();
				ptr->P_DDC = actual_position;
				lock->unlock();
			});

			// timer is called every 100ms to send trajectory setpoints and associated info
			auto timer_callback = [this, ptr, lock]() -> void {

				// offboard_control_mode needs to be paired with trajectory_setpoint
				this->publish_trajectory_setpoint(ptr, lock);

			};
			timer_ = this->create_wall_timer(100ms, timer_callback);
		}

	private:
		// needed rclcpp ptr stuff
		rclcpp::TimerBase::SharedPtr timer_;

		rclcpp::Publisher<geometry::msgs::point>::SharedPtr setpoint_publisher_;
		rclcpp::Subscription<geometry::msgs::point>::SharedPtr vehicle_state_subscriber_;

		std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

		// series of functions needed to operate the publishers
		void publish_trajectory_setpoint(BMN::Var_Transfer* ptr, std::mutex* lock);
	};
}

#endif