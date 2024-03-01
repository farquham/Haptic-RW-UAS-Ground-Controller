#ifndef PX4Comms_H
#define PX4Comms_H

// include eigen dense and sparse and alse the quadcopter classes
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <chrono>
#include <cstdio>
#include <mutex>

// px4 stuff needed for communication with IRL drone
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include "commsmsgs/msg/Rpicommspub.hpp"
#include "commsmsgs/msg/Brimpub.hpp"
#include "commsmsgs/msg/Rbquadsimpub.hpp"

// some nanespace stuff
using namespace std::chrono;
using namespace std::chrono_literals;

namespace RPI {
	// class for offboard control which starts a ROS2 node
	class rpicomms : public rclcpp::Node {
	public:
		rpicomms() : Node("rpi_comms")
		{
			// subscribers for recieving ddc from bmn comp
			brim_subscriber_ = this->create_subscription<commsmsgs::msg::brimpub>("/GC/out/brim", 10, std::bind(&rpicomms::brim_callback, [this], std::placeholders::_1));

			// publishers for send ddc to rpi companion comp
			setpoint_publisher_ = this->create_publisher<geometry_msgs::msgs::point>("/rpi/in/ext_cmdPoint", 10);

			// state subscriber for recieving IRL drone position, velocity and acceleration from rpi companion comp
			vehicle_state_subscriber_ = this->create_subscription<geometry_msgs::msgs::point>("/rpi/out/localPoint", 10, std::bind(&rpicomms::vehicle_state_callback, [this], std::placeholders::_1));
			
			// timer is called every 100ms to send trajectory setpoints and associated info
			auto timer_callback = [this]() -> void {

				// offboard_control_mode needs to be paired with trajectory_setpoint
				this->publish_trajectory_setpoint();

			};

			// init node vars
			// initrpicomms();

			timer_ = this->create_wall_timer(1ms, timer_callback);
		}

	private:
		// needed rclcpp ptr stuff
		rclcpp::Publisher<geometry::msgs::point>::SharedPtr setpoint_publisher_;
		rclcpp::Subscription<geometry::msgs::point>::SharedPtr vehicle_state_subscriber_;
		rclcpp::Subscription<commsmsgs::msg::brimpub>::SharedPtr brim_subscriber_;
		//rclcpp::Subscription<commsmsgs::msg::rbquadsimpub>::SharedPtr rbquad_subscriber_;

		rclcpp::TimerBase::SharedPtr timer_;

		std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

		// series of functions needed to operate the publisher
		void publish_trajectory_setpoint();
		void vehicle_state_callback(const geometry::msgs::point::UniquePtr & msg);
		void brim_callback(const commsmsgs::msg::brimpub::UniquePtr & msg);

		Eigen::Vector3d drone_position_cmd;
		Eigen::Vector3d drone_position_actual;
	};
}

#endif