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

namespace BMN {
	class bmnav : public rclcpp:Node {
		public:
			bmnav(float freqbmn, double k_b, double k_i, double d_i, double v_s, double p_s, double b_r, double c_r, double a_d, double v_l, double f_s, double flimx, double flimy, double flimz, double phin_max, double vmaxchange, double PSchange, double VSchange) : Node("bm_nav")
			{
				brim_subscriber_ = this->create_subscription<commsmsgs::msg::Brimpub>("/GC/out/brim", 10, std::bind(&bmnav::brim_callback, [this], std::placeholders::_1));
				rbquadsim_subscriber_ = this->create_subscription<commsmsgs::msg::Rbquadsimpub>("/GC/out/rbquadsim", 10, std::bind(&bmnav::rbquadsim_callback, [this], std::placeholders::_1));

				bmn_publisher_ = this->create_publisher<commsmsgs::msg::Bmnpub>("/GC/out/bmn", 10);

				auto timer_callback = [this]() -> void {
					// what ever code to run every timer iteration
					this->BMNstep();
				}

				// add params
				initbmn(freqbmn, k_b, k_i, d_i, v_s, p_s, b_r, c_r, a_d, v_l, f_s, flimx, flimy, flimz, phin_max, vmaxchange, PSchange, VSchange);

				start_time = clocky::now();

				timer_pub_ = this->create_wall_timer(1ms, timer_callback);
			}

		private:
			// subscibers and publishers
			rclcpp::Subscription<commsmsgs::msg::Brimpub>::SharedPtr brim_subscriber_;
			rclcpp::Subscription<commsmsgs::msg::Rbquadsimpub>::SharedPtr rbquadsim_subscriber_;
			rclcpp::Publisher<commsmsgs::msg::Bmnpub>::SharedPtr bmn_publisher_;

			rclcpp::TimerBase::SharedPtr timer_pub_;

			std::atomic<uint64_t> timestamp_;

			// initialize bmn class
			void initbmn(float fbmn, double k_b, double k_i, double d_i, double v_s, double p_s, double b_r, double c_r, double a_d, double v_l, double f_s, double flimx, double flimy, double flimz, double phin_max, double vmaxchange, double PSchange, double VSchange);
			// setting up the inverse
			std::string inverse3_setup();
			// safety function to center the end effector
			void centerDevice();
			// the main function for the bmn simulation which loops until the user stops it
			void BMNstep();

			// callback for the brim subscriber
			void brim_callback(const commsmsgs::msg::Brimpub::UniquePtr & msg);
			// callback for the rbquadsim subscriber
			void rbquadsim_callback(const commsmsgs::msg::Rbquadsimpub::UniquePtr & msg);

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
			API::Devices::Inverse3 Inverse_object;
			double h;
			EIgen::Vector3d w_c;
			double rest;
			// inverse data fetching and handling
    		API::Devices::Inverse3::EndEffectorStateResponse state;
    		API::Devices::Inverse3::EndEffectorForceRequest requested;

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

			auto start_time = clocky::now();
	}
}

#endif