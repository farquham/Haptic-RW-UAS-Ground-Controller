#ifndef BRIM_H
#define BRIM_H

// includes
#include <iostream>
#include <chrono>
#include <cstdio>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/QR>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "commsmsgs/msg/brimpub.hpp"
#include "commsmsgs/msg/bmnpub.hpp"
#include "commsmsgs/msg/rbquadsimpub.hpp"

namespace BRIM {
	class brim : public rclcpp:Node {
	public:
		brim(float freqrim, int fcom1, int fcom2, int rim_type, float xlim, float ylim, float zlim, float dxlim, float dylim, float dzlim) : Node("brim_node")
		{
			bmn_subscriber_ = this->create_subscription<commsmsgs::msg::Bmnpub>("/GC/out/bmn", 10, std::bind(&brim::bmn_callback, [this], std::placeholders::_1));
			rbquadsim_subscriber_ = this->create_subscription<commsmsgs::msg::Rbquadsimpub>("/GC/out/rbquadsim", 10, std::bind(&brim::rbquadsim_callback, [this], std::placeholders::_1));

			brim_publisher_ = this->create_publisher<commsmsgs::msg::Brimpub>("/GC/out/brim", 10);

			auto timer_callback = [this]() -> void {
				// what ever code to run every timer iteration
				this->BRIMstep();
			}

			// add params
			initbrim(freqrim, fcom1, fcom2, rim_type, xlim, ylim, zlim, dxlim, dylim, dzlim);

			// wait for a few ms for variables to finish initializing
			// I dont know for some reason we need a pause here
			for (int i = 0; i < 100; i++) {}

			start_time = clocky::now();
			loop_time = clocky::now();

			timer_pub_ = this->create_wall_timer(1ms, timer_callback);
		}
	private:
		// subscibers and publishers
		rclcpp::Subscription<commsmsgs::msg::Bmnpub>::SharedPtr bmn_subscriber_;
		rclcpp::Subscription<commsmsgs::msg::Rbquadsimpub>::SharedPtr rbquadsim_subscriber_;
		rclcpp::Publisher<commsmsgs::msg::Brimpub>::SharedPtr brim_publisher_;

		rclcpp::TimerBase::SharedPtr timer_pub_;

		std::atomic<uint64_t> timestamp_;

		// BRIM constructor
		initbrim(float frim, int fcom1, int fcom2, int rim_type, float xlim, float ylim, float zlim, float dxlim, float dylim, float dzlim);
		// BRIM loop starter
		void BRIMstep();

		// callback for the bmn subscriber
		void bmn_callback(const commsmsgs::msg::Bmnpub::UniquePtr & msg);
		// callback for the rbquadsim subscriber
		void rbquadsim_callback(const commsmsgs::msg::Rbquadsimpub::UniquePtr & msg);

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
		void bmn_update(double* r_type, Eigen::Vector3d* phin_list, Eigen::Vector3d* dot_phin_list, Eigen::Vector3d* lambda_tilde, Eigen::Vector3d* lambda_i, Eigen::Vector3d* R_vec_est, Eigen::Matrix3d* M_tilde_inv, double h, Eigen::Vector3d* dx_lims, Eigen::Vector3d* x_lims);
		// sparse matrix helpers
		void sparse_fill(Eigen::SparseMatrix<double>* tb_filled, Eigen::Matrix3d* t_fill, int row, int col);
		void sparse_replace(Eigen::SparseMatrix<double>* tb_replaced, Eigen::Matrix3d* t_replace, int row, int col);
		// lim helper
		void Vec3Lim(Eigen::Vector3d* vec, Eigen::Vector3d* lim);
		// runge kutta 4th order integrator for use throughout simulation
		void RK4_update(double* xn, double* xn_dot, double* h);
		void RK4_vec_update(Eigen::Vector3d* xn, Eigen::Vector3d xn_dot, double h);	

		// ros msg translation helpers
		void msg_to_matrix(std_msgs::msg::Float64MultiArray min, Eigen::SparseMatrix<double>* mout);
		
		
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
		Eigen::Vector3d DDP;
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

		Eigen::Matrix<double, 1, 42> tempvb;

		std::chrono::duration<double> start_time;
		std::chrono::duration<double> loop_time;
	};
}

#endif