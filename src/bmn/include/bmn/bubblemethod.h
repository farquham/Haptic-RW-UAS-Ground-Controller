#ifndef BubbleMethod_H
#define BubbleMethod_H

// include eigen dense and sparse and alse the quadcopter and haply stuff
#include <iostream>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "../../deps/HardwareAPI/include/HardwareAPI.h"

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include "commsmsgs/msg/brimpub.hpp"
#include "commsmsgs/msg/bmnpub.hpp"
#include "commsmsgs/msg/rbquadsimpub.hpp"
#include "commsmsgs/msg/guicontrols.hpp"

typedef std::chrono::high_resolution_clock clocky;
using namespace std::chrono_literals;

namespace API = Haply::HardwareAPI;

namespace BMN {
	class bmnav : public rclcpp::Node {
		public:
			bmnav() : Node("bmn_node")
			{
				this->declare_parameter("freqbmn", 1000.0);
				this->declare_parameter("k_b", 0.0);
				this->declare_parameter("k_i", 0.0);
				this->declare_parameter("d_i", 0.0);
				this->declare_parameter("v_s", 0.0);
				this->declare_parameter("p_s", 0.0);
				this->declare_parameter("b_r", 0.0);
				this->declare_parameter("c_r", 0.0);
				this->declare_parameter("a_d", 0.0);
				this->declare_parameter("v_l", 0.0);
				this->declare_parameter("f_s", 0.0);
				this->declare_parameter("flimx", 0.0);
				this->declare_parameter("flimy", 0.0);
				this->declare_parameter("flimz", 0.0);
				this->declare_parameter("phin_max", 0.0);
				this->declare_parameter("vmaxchange", 0.0);
				this->declare_parameter("PSchange", 0.0);
				this->declare_parameter("VSchange", 0.0);

				float freqbmn = this->get_parameter("freqbmn").as_double();
				double k_b = this->get_parameter("k_b").as_double();
				double k_i = this->get_parameter("k_i").as_double();
				double d_i = this->get_parameter("d_i").as_double();
				double v_s = this->get_parameter("v_s").as_double();
				double p_s = this->get_parameter("p_s").as_double();
				double b_r = this->get_parameter("b_r").as_double();
				double c_r = this->get_parameter("c_r").as_double();
				double a_d = this->get_parameter("a_d").as_double();
				double v_l = this->get_parameter("v_l").as_double();
				double f_s = this->get_parameter("f_s").as_double();
				double flimx = this->get_parameter("flimx").as_double();
				double flimy = this->get_parameter("flimy").as_double();
				double flimz = this->get_parameter("flimz").as_double();
				double phin_max = this->get_parameter("phin_max").as_double();
				double vmaxchange = this->get_parameter("vmaxchange").as_double();
				double PSchange = this->get_parameter("PSchange").as_double();
				double VSchange = this->get_parameter("VSchange").as_double();

				brim_subscriber_ = this->create_subscription<commsmsgs::msg::Brimpub>("/GC/out/brim", 10, std::bind(&bmnav::brim_callback, this, std::placeholders::_1));
				rbquadsim_subscriber_ = this->create_subscription<commsmsgs::msg::Rbquadsimpub>("/GC/out/rbquadsim", 10, std::bind(&bmnav::rbquadsim_callback, this, std::placeholders::_1));
				guicontrols_subscriber_ = this->create_subscription<commsmsgs::msg::Guicontrols>("/GC/internal/guictrls", 10, std::bind(&bmnav::guicontrols_callback, this, std::placeholders::_1));

				bmn_publisher_ = this->create_publisher<commsmsgs::msg::Bmnpub>("/GC/out/bmn", 10);

				// inverse startup
    			comm = BMN::bmnav::inverse3_setup();
    			API::IO::SerialStream stream{ comm.c_str() };
    			API::Devices::Inverse3 Inverse_object{&stream};

				auto timey_callback = [this, Inverse_object]() -> void {
					// what ever code accel_stampedto run every timer iteration
					this->timer_callback(Inverse_object);
				};

				// add params
				initbmn(freqbmn, k_b, k_i, d_i, v_s, p_s, b_r, c_r, a_d, v_l, f_s, flimx, flimy, flimz, phin_max, vmaxchange, PSchange, VSchange, Inverse_object);

				start_time = clocky::now();

				auto time = 1000ms / freqbmn;

				timer_pub_ = this->create_wall_timer(time, timey_callback);
			}

		private:
			void timer_callback(API::Devices::Inverse3 Inverse_object) {
				// what ever code to run every timer iteration
				//if (run) {
					this->BMNstep(Inverse_object_ptr);
				//}
				commsmsgs::msg::Bmnpub msg{};
				msg.header.stamp = this->now();
				msg.running = run;
				msg.interface_force_list.x = iforce_list[0];
				msg.interface_force_list.y = iforce_list[1];
				msg.interface_force_list.z = iforce_list[2];
				msg.desired_drone_position.x = D_D_C[0];
				msg.desired_drone_position.y = D_D_C[1];
				msg.desired_drone_position.z = D_D_C[2];
				msg.bmn_freq = freq;
				RCLCPP_INFO(this->get_logger(), "Publishing interface force: x: %f y: %f z: %f", iforce_list[0], iforce_list[1], iforce_list[2]);
				RCLCPP_INFO(this->get_logger(), "Publishing desired drone position: x: %f y: %f z: %f", D_D_C[0], D_D_C[1], D_D_C[2]);
    			bmn_publisher_->publish(msg);
			}
			// subscibers and publishers
			rclcpp::Subscription<commsmsgs::msg::Brimpub>::SharedPtr brim_subscriber_;
			rclcpp::Subscription<commsmsgs::msg::Rbquadsimpub>::SharedPtr rbquadsim_subscriber_;
			rclcpp::Subscription<commsmsgs::msg::Guicontrols>::SharedPtr guicontrols_subscriber_;
			rclcpp::Publisher<commsmsgs::msg::Bmnpub>::SharedPtr bmn_publisher_;

			rclcpp::TimerBase::SharedPtr timer_pub_;

			std::atomic<uint64_t> timestamp_;

			// initialize bmn class
			void initbmn(float fbmn, double k_b, double k_i, double d_i, double v_s, double p_s, double b_r, double c_r, double a_d, double v_l, double f_s, double flimx, double flimy, double flimz, double phin_max, double vmaxchange, double PSchange, double VSchange, API::Devices::Inverse3 Inverse_object);
			// setting up the inverse
			std::string inverse3_setup();
			// safety function to center the end effector
			void centerDevice(API::Devices::Inverse3 Inverse_object);
			// the main function for the bmn simulation which loops until the user stops it
			void BMNstep(API::Devices::Inverse3* Inverse_objecto);

			// callback for the brim subscriber
			void brim_callback(const commsmsgs::msg::Brimpub::UniquePtr & msg);
			// callback for the rbquadsim subscriber
			void rbquadsim_callback(const commsmsgs::msg::Rbquadsimpub::UniquePtr & msg);
			// callback for the guicontrols subscriber
			void guicontrols_callback(const commsmsgs::msg::Guicontrols::UniquePtr & msg);

			// helper functions
			// function to calculate the interface force for given phins and dot_phins
			void force_interface(double* stiff, double* damp, double* act_dis, Eigen::Matrix<double, 3, 1>* phins, Eigen::Matrix<double, 3, 1>* dot_phins, Eigen::Vector3d* force, Eigen::Vector3d* force_list);
			// function to calculate the bubble force for given EE postion
			void force_restitution(Eigen::Vector3d* center, Eigen::Vector3d* device_pos, double* radius, double* stiffness, Eigen::Vector3d* force);
			// function to limit the forces sent to the inverse
			void force_scaling(Eigen::Vector3d* Force, Eigen::Vector3d* lims, double* scale);
			// function to calculate the velocity applied to the bubble for a given EE position
			void velocity_applied(Eigen::Vector3d* center, Eigen::Vector3d* device_pos, double* radius, double scaling, Eigen::Vector3d* Va);
			// function to check if the EE is near its workspace center
			bool start_check(Eigen::Vector3d* center, Eigen::Vector3d* device_pos, double* radiusdead);
			// function to check if the EE is inside the bubble
			bool pos_check(Eigen::Vector3d* center, Eigen::Vector3d* device_pos, double* bound_rad);
			// runge kutta 4th order integrator for use throughout simulation
			void RK4_update(double* xn, double* xn_dot, double* h);
			void RK4_vec_update(Eigen::Vector3d* xn, Eigen::Vector3d xn_dot, double h);


			// fields for class
			// inverse stuff
			std::string comm;
			API::Devices::Inverse3* Inverse_object_ptr;
			double h;
			Eigen::Vector3d w_c;
			double rest;
			// inverse data fetching and handling
    		//API::Devices::Inverse3::EndEffectorStateResponse state;
    		//API::Devices::Inverse3::EndEffectorForceRequest requested;

			// looping vars
			Eigen::Vector3d raw_positions;
			Eigen::Vector3d velocities;
			Eigen::Vector3d positions;
			Eigen::Vector3d abs_positions;
			Eigen::Vector3d forces;
			Eigen::Vector3d Va;
			Eigen::Vector3d D_D_C;
			Eigen::Vector3d V_B_C;
			Eigen::Vector3d A_D_C;
			Eigen::Vector3d phins;
			Eigen::Vector3d dot_phins;
			Eigen::Vector3d rforces;
			Eigen::Vector3d iforces;
			Eigen::Vector3d iforce_list;

			// looping params
			int count;
			int i;
			double magVa;
			double freq;

			double stiff_k;
			double stiff_ki;
			double damp_b;
			double damp_bi;
			double stiff_bk;
			double bub_rad;
			double con_rad;
			double act_dis;
			double act_dis_in;
			double maxVa;
			double maxVai;
			double phinmax;
			double vachange;
			double posspringadj;
			double velspringadj;
			double p_scale;
			double v_scale;

			bool run;
			bool boundary;
			bool ncon;
			bool con;
			bool precon;
			bool postcon;

			Eigen::Vector3d flims;
			double fscale;

			std::chrono::_V2::system_clock::time_point start_time;
	};
}

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BMN::bmnav>());
	rclcpp::shutdown();
	return 0;
}

#endif