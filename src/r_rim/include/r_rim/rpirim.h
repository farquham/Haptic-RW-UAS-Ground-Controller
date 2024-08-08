#ifndef PRIM_H
#define PRIM_H

// include eigen dense and sparse and alse the quadcopter classes
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/QR>
#include <chrono>
#include <cstdio>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "commsmsgs/msg/rrimpub.hpp"
#include "commsmsgs/msg/rpicommspub.hpp"
#include "commsmsgs/msg/rbquadsimpub.hpp"
#include "commsmsgs/msg/guicontrols.hpp"
#include "commsmsgs/msg/logsetup.hpp"

typedef std::chrono::high_resolution_clock clocky;
using namespace std::chrono_literals;

namespace PRIM {
	class prim : public rclcpp::Node {
	public:
		prim() : Node("prim_node")
		{
			this->declare_parameter("freqrim", 1000.0);
			this->declare_parameter("fcom1", 0);
			this->declare_parameter("fcom2", 0);
			this->declare_parameter("rim_type", 0);
			this->declare_parameter("xlim", 0.0);
			this->declare_parameter("ylim", 0.0);
			this->declare_parameter("zlim", 0.0);
			this->declare_parameter("dxlim", 0.0);
			this->declare_parameter("dylim", 0.0);
			this->declare_parameter("dzlim", 0.0);

			float freqrim = this->get_parameter("freqrim").as_double();
			int fcom1 = this->get_parameter("fcom1").as_int();
			int fcom2 = this->get_parameter("fcom2").as_int();
			int rim_type = this->get_parameter("rim_type").as_int();
			float xlim = this->get_parameter("xlim").as_double();
			float ylim = this->get_parameter("ylim").as_double();
			float zlim = this->get_parameter("zlim").as_double();
			float dxlim = this->get_parameter("dxlim").as_double();
			float dylim = this->get_parameter("dylim").as_double();
			float dzlim = this->get_parameter("dzlim").as_double();

			rpi_subscriber_ = this->create_subscription<commsmsgs::msg::Rpicommspub>("/GC/out/rpicomms", 10, std::bind(&prim::rpi_callback, this, std::placeholders::_1));
			rbquadsim_subscriber_ = this->create_subscription<commsmsgs::msg::Rbquadsimpub>("/GC/out/rbquadsim", 10, std::bind(&prim::rbquadsim_callback, this, std::placeholders::_1));
			guicontrols_subscriber_ = this->create_subscription<commsmsgs::msg::Guicontrols>("/GC/internal/guictrls", 10, std::bind(&prim::guicontrols_callback, this, std::placeholders::_1));
			logs_subscriber_ = this->create_subscription<commsmsgs::msg::Logsetup>("/GC/internal/logsetup", 10, std::bind(&prim::logsetup_callback, this, std::placeholders::_1));

			prim_publisher_ = this->create_publisher<commsmsgs::msg::Rrimpub>("/GC/out/prim", 10);

			// auto timer_callback = [this]() -> void {
			// 	// what ever code to run every timer iteration
			// 	this->PRIMstep();
			// }

			// add params
			initprim(freqrim, fcom1, fcom2, rim_type, xlim, ylim, zlim, dxlim, dylim, dzlim);

			// wait for a few ms for variables to finish initializing
			// I dont know for some reason we need a pause here
			for (int i = 0; i < 100; i++) {}

			RCLCPP_INFO(this->get_logger(), "PRIM Node Started");

			start_time = clocky::now();

			auto time = 1000ms / freqrim;

			timer_pub_ = this->create_wall_timer(time, std::bind(&PRIM::prim::timer_callback, this));
		}
	private:
		void timer_callback() {
			// what ever code to run every timer iteration
			// publishs the PRIM data
			commsmsgs::msg::Rrimpub msg{};
			msg.header.stamp = this->now();
			msg.running = run;
			if (run) {
				//this->PRIMstep();
				// msg.phin_list.x = phin_list[0];
				// msg.phin_list.y = phin_list[1];
				// msg.phin_list.z = phin_list[2];
				// msg.phin_dot_list.x = dot_phin_list[0];
				// msg.phin_dot_list.y = dot_phin_list[1];
				// msg.phin_dot_list.z = dot_phin_list[2];
				msg.actual_drone_position.x = ADP[0];
				msg.actual_drone_position.y = ADP[1];
				msg.actual_drone_position.z = ADP[2];
				msg.rbsim_position.x = DP[0];
				msg.rbsim_position.y = DP[1];
				msg.rbsim_position.z = DP[2];
				//RCLCPP_INFO(this->get_logger(), "Publishing actual drone position: x: %f y: %f z: %f", ADP[0], ADP[1], ADP[2]);
				//RCLCPP_INFO(this->get_logger(), "Publishing rbsim position: x: %f y: %f z: %f", DP[0], DP[1], DP[2]);
				// msg.rrim_freq = freq;
				// msg.rrim_count = count;
				// msg.rrim_time = loop_time.count();
			}
			prim_publisher_->publish(msg);
		}
		// subscibers and publishers
		rclcpp::Subscription<commsmsgs::msg::Rpicommspub>::SharedPtr rpi_subscriber_;
		rclcpp::Subscription<commsmsgs::msg::Rbquadsimpub>::SharedPtr rbquadsim_subscriber_;
		rclcpp::Publisher<commsmsgs::msg::Rrimpub>::SharedPtr prim_publisher_;
		rclcpp::Subscription<commsmsgs::msg::Guicontrols>::SharedPtr guicontrols_subscriber_;
		rclcpp::Subscription<commsmsgs::msg::Logsetup>::SharedPtr logs_subscriber_;

		rclcpp::TimerBase::SharedPtr timer_pub_;

		std::atomic<uint64_t> timestamp_;

		// BRIM constructor
		void initprim(float frim, int fcom1, int fcom2, int rim_type, float xlim, float ylim, float zlim, float dxlim, float dylim, float dzlim);
		// BRIM loop starter
		void PRIMstep();

		// callback for the bmn subscriber
		void rpi_callback(const commsmsgs::msg::Rpicommspub::UniquePtr & msg);
		// callback for the rbquadsim subscriber
		void rbquadsim_callback(const commsmsgs::msg::Rbquadsimpub::UniquePtr & msg);
		// callback for the guicontrols subscriber
		void guicontrols_callback(const commsmsgs::msg::Guicontrols::UniquePtr & msg);
		// * callback for the logsetup subscriber to set the correct rim type
		void logsetup_callback(const commsmsgs::msg::Logsetup::UniquePtr & msg);

		// helper methods for BRIM
		// starts all the matrices
		void init_mats();
		// calculates the interface jacobian
		void Interface_Jacobian(Eigen::Vector3d* DP, Eigen::Vector3d* DDP, Eigen::Vector3d* R_vec, Eigen::Matrix3d* R_tilde_mat, Eigen::Vector3d* phin_list, Eigen::SparseMatrix<double>* Ai, Eigen::SparseMatrix<double>* Ai_old, Eigen::SparseMatrix<double>* Ai_dot, double h);
		// sets up the rigid body system matrices
		void rb_setup(Eigen::SparseMatrix<double>* Ac, Eigen::SparseMatrix<double>* M_hat_inv, Eigen::SparseMatrix<double>* Pc_hat, Eigen::SparseMatrix<double>* I, Eigen::SparseMatrix<double>* IPMi, Eigen::Matrix3d* M_temp_offset);
		// updates the rigid body system projection calculations when needed
		void rb_update(Eigen::Vector3d* DP, Eigen::Vector3d* DDP, Eigen::Vector3d* R_vec, Eigen::Matrix3d* R_tilde_mat, Eigen::Vector3d* phin_list, Eigen::Vector3d* dot_phin_list, Eigen::SparseMatrix<double>* Ai, Eigen::SparseMatrix<double>* Ai_old, Eigen::SparseMatrix<double>* Ai_dot, Eigen::SparseMatrix<double>* IPMi, Eigen::Matrix<double, 42, 1>* v_g, double h, Eigen::Vector3d* Drag, Eigen::Vector3d* Gravity, Eigen::Matrix3d* M_temp_offset, Eigen::Matrix3d* M_tilde, Eigen::Matrix3d* M_tilde_inv, Eigen::Matrix<double, 42, 1>* f_ext, Eigen::Vector3d* lambda_tilde, Eigen::Vector3d* lambda_i, Eigen::SparseMatrix<double>* Pc_hat, Eigen::Matrix<double, 42, 1>* v_g_old, double h_com1, Eigen::Vector3d* Drone_Interaction);
		// updates the bubble method system when needed
		void fast_update(double* r_type, Eigen::Vector3d* phin_list, Eigen::Vector3d* dot_phin_list, Eigen::Vector3d* lambda_tilde, Eigen::Vector3d* lambda_i, Eigen::Vector3d* R_vec_est, Eigen::Matrix3d* M_tilde_inv, double h, Eigen::Vector3d* dx_lims, Eigen::Vector3d* x_lims);
		// sparse matrix helpers
		void sparse_fill(Eigen::SparseMatrix<double>* tb_filled, Eigen::Matrix3d* t_fill, int row, int col);
		void sparse_replace(Eigen::SparseMatrix<double>* tb_replaced, Eigen::Matrix3d* t_replace, int row, int col);
		// lim helper
		void Vec3Lim(Eigen::Vector3d* vec, Eigen::Vector3d* lim);
		// runge kutta 4th order integrator for use throughout simulation
		void RK4_update(double* xn, double* xn_dot, double* h);
		void RK4_vec_update(Eigen::Vector3d* xn, Eigen::Vector3d xn_dot, double h);	

		// ros msg translation helpers
		void msg_to_matrix(std::array<double,768> min, Eigen::SparseMatrix<double>* mout);		
		
		// RIM variables
		// sparse matrices used for rigidbody projection calculations
		Eigen::SparseMatrix <double> Ac;
		Eigen::SparseMatrix <double> M_hat_inv;
		Eigen::SparseMatrix <double> Pc_hat;
		Eigen::SparseMatrix <double> Ai;
		Eigen::SparseMatrix <double> Ai_old;
		Eigen::SparseMatrix <double> Ai_dot;
		Eigen::SparseMatrix <double> I;
		Eigen::SparseMatrix <double> IPMi;
		// vectors used for rigidbody projection calculations
		Eigen::Matrix <double, 42, 1> v_g;
		Eigen::Matrix <double, 42, 1> v_g_old;

		// projected vectors used in the RIM calclations
		Eigen::Matrix <double, 3, 3> M_tilde;
		Eigen::Matrix <double, 3, 3> M_tilde_inv;

		Eigen::Matrix <double, 3, 1> lambda_tilde;
		Eigen::Matrix <double, 3, 1> lambda_i;

		Eigen::Matrix<double, 3, 1> phin_list;
		Eigen::Matrix<double, 3, 1> dot_phin_list;

		Eigen::Matrix<double, 42, 1> f_ext;

		Eigen::Matrix <double, 3, 3> M_temp_offset;

		// variables used in the prediction part of RIM
		Eigen::Vector3d R_vec;
		Eigen::Vector3d R_vec_est;
		Eigen::Matrix3d R_tilde_mat;
		Eigen::Vector3d ADP;
		Eigen::Vector3d DP;

		// limit vars
		Eigen::Vector3d lims;
		Eigen::Vector3d dlims;

		Eigen::Vector3d fd;
		Eigen::Vector3d fg;
		Eigen::Vector3d fi;

		// helper vars for system setup
		double frim;
		double h;
		double h_com1;
		double r_type;

		int count, i;
		double freq;

		bool run;

		Eigen::Matrix<double, 1, 42> tempvb;

		std::chrono::_V2::system_clock::time_point start_time;
		std::chrono::duration<double> loop_time;
	};
}

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PRIM::prim>());
	rclcpp::shutdown();
	return 0;
}

#endif