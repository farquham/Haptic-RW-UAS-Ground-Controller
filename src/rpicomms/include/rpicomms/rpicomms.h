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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include "commsmsgs/msg/rpicommspub.hpp"
#include "commsmsgs/msg/rrimpub.hpp"
#include "commsmsgs/msg/rbquadsimpub.hpp"
#include "commsmsgs/msg/guicontrols.hpp"

// some nanespace stuff
using namespace std::chrono;
using namespace std::chrono_literals;
typedef std::chrono::high_resolution_clock clocky;

namespace RPI {
	// class for offboard control which starts a ROS2 node
	class rpicomms : public rclcpp::Node {
	public:
		rpicomms() : Node("rpi_comms")
		{
			this -> declare_parameter("desired_frequency", 1000.0);
			float desired_frequency = this->get_parameter("desired_frequency").as_double();

			// subscribers for recieving ddc from bmn comp
			rrim_subscriber_ = this->create_subscription<commsmsgs::msg::Rrimpub>("/GC/out/prim", 10, std::bind(&rpicomms::prim_callback, this, std::placeholders::_1));
			guicontrols_subscriber_ = this->create_subscription<commsmsgs::msg::Guicontrols>("/GC/internal/guictrls", 10, std::bind(&rpicomms::guicontrols_callback, this, std::placeholders::_1));

			// publishers for send ddc to rpi companion comp
			setpoint_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/rpi/in/ext_cmdPoint", 10);

			// state subscriber for recieving IRL drone position, velocity and acceleration from rpi companion comp
			vehicle_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/rpi/out/vehicle_pose", 10, std::bind(&rpicomms::vehicle_pose_callback, this, std::placeholders::_1));
			vehicle_twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/rpi/out/vehicle_velocity", 10, std::bind(&rpicomms::vehicle_twist_callback, this, std::placeholders::_1));
			vehicle_accel_subscriber_ = this->create_subscription<geometry_msgs::msg::AccelStamped>("/rpi/out/vehicle_acceleration", 10, std::bind(&rpicomms::vehicle_accel_callback, this, std::placeholders::_1));

			// publisher for sending IRL drone position, velocity and acceleration to the rest of the system
			rpicomms_publisher_ = this->create_publisher<commsmsgs::msg::Rpicommspub>("/GC/out/rpicomms", 10);
			
			// // timer is called every 100ms to send trajectory setpoints and associated info
			// auto timer_callback = [this]() -> void {

			// 	// trajectory_setpoint
			// 	this->publish_trajectory_setpoint();
			// 	// vehicle_state
			// 	this->publish_vehicle_state();

			// };

			// init node vars
			// initrpicomms();
			run = false;

			RCLCPP_INFO(this->get_logger(), "RPI Comms Node Started");

			// desired frequency to run the timer at
			auto time = 1000ms / desired_frequency;

			timer_ = this->create_wall_timer(time, std::bind(&RPI::rpicomms::timer_callback, this));
		}

	private:
		void timer_callback() {
			// what ever code to run every timer iteration
			if (run) {
				this->publish_trajectory_setpoint();
			}
			this->publish_vehicle_state();
		}
		// needed rclcpp ptr stuff
		rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr setpoint_publisher_;
		rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vehicle_pose_subscriber_;
		rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vehicle_twist_subscriber_;
		rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr vehicle_accel_subscriber_;
		rclcpp::Subscription<commsmsgs::msg::Rrimpub>::SharedPtr rrim_subscriber_;
		rclcpp::Subscription<commsmsgs::msg::Guicontrols>::SharedPtr guicontrols_subscriber_;
		//rclcpp::Subscription<commsmsgs::msg::rbquadsimpub>::SharedPtr rbquad_subscriber_;
		rclcpp::Publisher<commsmsgs::msg::Rpicommspub>::SharedPtr rpicomms_publisher_;

		rclcpp::TimerBase::SharedPtr timer_;

		std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

		// series of functions needed to operate the publisher
		void publish_trajectory_setpoint();
		void vehicle_pose_callback(const geometry_msgs::msg::PoseStamped::UniquePtr & msg);
		void vehicle_twist_callback(const geometry_msgs::msg::TwistStamped::UniquePtr & msg);
		void vehicle_accel_callback(const geometry_msgs::msg::AccelStamped::UniquePtr & msg);
		void prim_callback(const commsmsgs::msg::Rrimpub::UniquePtr & msg);
		void publish_vehicle_state();
		void guicontrols_callback(const commsmsgs::msg::Guicontrols::UniquePtr & msg);

		Eigen::Vector3d drone_position_cmd;
		Eigen::Vector3d drone_position_actual;
		Eigen::Vector4d drone_orientation_actual;
		Eigen::Vector3d drone_velocity_actual;
		Eigen::Vector3d drone_acceleration_actual;

		bool run;
	};
}

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RPI::rpicomms>());
	rclcpp::shutdown();
	return 0;
}

#endif