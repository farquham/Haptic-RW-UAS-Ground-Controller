#ifndef BubbleMethod_H
#define BubbleMethod_H

// include eigen dense and sparse and alse the quadcopter and haply stuff
#include <iostream>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <thread>
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
	struct data_pipe {
		//out
		double frequency;
		Eigen::Vector3d desired_drone_position;
		Eigen::Vector3d interface_force_list;
		//in
		Eigen::Vector3d phin_distance;
		Eigen::Vector3d phin_velocity;
		Eigen::Vector3d simulated_drone_position;
		bool contact;
		bool pre_contact;
		bool post_contact;
		bool no_contact;
		bool running;

		// params
		float freqbmn;
		double k_b;
		double k_i;
		double d_i;
		double v_s;
		double p_s;
		double b_r;
		double c_r;
		double a_d;
		double v_l;
		double f_s;
		double flimx;
		double flimy;
		double flimz;
		double phin_max;
		double vmaxchange;
		double PSchange;
		double VSchange;
	};
	class bubblemethodnavigation {
		public:
			// initialize bmn class
			bubblemethodnavigation(float fbmn, double k_b, double k_i, double d_i, double v_s, double p_s, double b_r, double c_r, double a_d, double v_l, double f_s, double flimx, double flimy, double flimz, double phin_max, double vmaxchange, double PSchange, double VSchange);
			// function to run inverse coonnection
			void Inverse_Connection(BMN::data_pipe* data, std::mutex* lock);
			
		private:
			// setting up the inverse
			std::string inverse3_setup();
			// safety function to center the end effector
			void centerDevice(API::Devices::Inverse3 Inverse_object);
			// the main function for the bmn simulation which loops until the user stops it
			void BMNstep(API::Devices::Inverse3 Inverse_object);
			
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

			// inverse stuff
			std::string comm;
			API::Devices::Inverse3* Inverse_object_ptr;
			double h;
			Eigen::Vector3d w_c;
			double rest;
			// inverse data fetching and handling
    		//API::Devices::Inverse3::EndEffectorStateResponse state;
    		//API::Devices::Inverse3::EndEffectorForceRequest requested;

			// class fields
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


void bmnsetup(BMN::data_pipe* ptr, std::mutex* lock);

namespace BMN {
	class bmnav : public rclcpp::Node {
		public:
			bmnav(BMN::data_pipe* ptr, std::mutex* lock) : Node("bmn_node")
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

				// param setup
				lock->lock();
				ptr->freqbmn = freqbmn;
				ptr->k_b = k_b;
				ptr->k_i = k_i;
				ptr->d_i = d_i;
				ptr->v_s = v_s;
				ptr->p_s = p_s;
				ptr->b_r = b_r;
				ptr->c_r = c_r;
				ptr->a_d = a_d;
				ptr->v_l = v_l;
				ptr->f_s = f_s;
				ptr->flimx = flimx;
				ptr->flimy = flimy;
				ptr->flimz = flimz;
				ptr->phin_max = phin_max;
				ptr->vmaxchange = vmaxchange;
				ptr->PSchange = PSchange;
				ptr->VSchange = VSchange;
				lock->unlock();

				RCLCPP_INFO(this->get_logger(), "BMN Node has been started");

				auto time = 1000ms / freqbmn;

				timer_pub_ = this->create_wall_timer(time, [this, ptr, lock]() {
					this->timer_callback(ptr, lock);
				});
			}

		private:
			void timer_callback(BMN::data_pipe* ptr, std::mutex* lock) {
				lock->lock();
				//out
				iforce_list = ptr->interface_force_list;
				D_D_C = ptr->desired_drone_position;
				freq = ptr->frequency;
				//in
				ptr->contact = con;
				ptr->pre_contact = precon;
				ptr->post_contact = postcon;
				ptr->no_contact = ncon;
				ptr->phin_distance = phins;
				ptr->phin_velocity = dot_phins;
				ptr->simulated_drone_position = A_D_C;
				ptr->running = run;
				lock->unlock();
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
				//RCLCPP_INFO(this->get_logger(), "Publishing interface force: x: %f y: %f z: %f", iforce_list[0], iforce_list[1], iforce_list[2]);
				//RCLCPP_INFO(this->get_logger(), "Publishing desired drone position: x: %f y: %f z: %f", D_D_C[0], D_D_C[1], D_D_C[2]);
    			bmn_publisher_->publish(msg);
			}
			// subscibers and publishers
			rclcpp::Subscription<commsmsgs::msg::Brimpub>::SharedPtr brim_subscriber_;
			rclcpp::Subscription<commsmsgs::msg::Rbquadsimpub>::SharedPtr rbquadsim_subscriber_;
			rclcpp::Subscription<commsmsgs::msg::Guicontrols>::SharedPtr guicontrols_subscriber_;
			rclcpp::Publisher<commsmsgs::msg::Bmnpub>::SharedPtr bmn_publisher_;

			rclcpp::TimerBase::SharedPtr timer_pub_;

			std::atomic<uint64_t> timestamp_;

			// callback for the brim subscriber
			void brim_callback(const commsmsgs::msg::Brimpub::UniquePtr & msg);
			// callback for the rbquadsim subscriber
			void rbquadsim_callback(const commsmsgs::msg::Rbquadsimpub::UniquePtr & msg);
			// callback for the guicontrols subscriber
			void guicontrols_callback(const commsmsgs::msg::Guicontrols::UniquePtr & msg);

			// fields for class

			// looping vars
			Eigen::Vector3d D_D_C;
			Eigen::Vector3d A_D_C;
			Eigen::Vector3d phins;
			Eigen::Vector3d dot_phins;
			Eigen::Vector3d iforce_list;

			// looping params
			double freq;

			bool run;
			bool ncon;
			bool con;
			bool precon;
			bool postcon;
	};
}

void bmnsetup(BMN::data_pipe* ptr, std::mutex* lock) {
	BMN::bubblemethodnavigation bmn_interface = BMN::bubblemethodnavigation(ptr->freqbmn, ptr->k_b, ptr->k_i, ptr->d_i, ptr->v_s, ptr->p_s, ptr->b_r, ptr->c_r, ptr->a_d, ptr->v_l, ptr->f_s, ptr->flimx, ptr->flimy, ptr->flimz, ptr->phin_max, ptr->vmaxchange, ptr->PSchange, ptr->VSchange);
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BMN setup complete");
	bmn_interface.Inverse_Connection(ptr, lock);
}

void nodespinup(BMN::data_pipe* ptr, std::mutex* lock) {
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BMN node spinning up");
	rclcpp::spin(std::make_shared<BMN::bmnav>(ptr, lock));
	rclcpp::shutdown();
}

int main(int argc, char * argv[]) {
	// mutlithreading stuff
	BMN::data_pipe data;
	BMN::data_pipe* ptr = &data;
	std::mutex lock;

	rclcpp::init(argc, argv);

	std::thread th1(bmnsetup, ptr, &lock);

	std::thread th2(nodespinup, ptr, &lock);

	th1.join();

	th2.join();

	return 0;
}

#endif