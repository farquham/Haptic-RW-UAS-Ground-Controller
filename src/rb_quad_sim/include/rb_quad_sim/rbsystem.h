#ifndef RBsystem_H
#define RBsystem_H

// include eigen dense and sparse and alse the quadcopter class
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include <chrono>
#include <cstdio>
#include <mutex>
#include "quadcopter.h"
#include "rb_helpers.h"

// include the ros2 stuff
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "commsmsgs/msg/brimpub.hpp"
#include "commsmsgs/msg/rrimpub.hpp"
#include "commsmsgs/msg/rbquadsimpub.hpp"

namespace RBsystem {		
	class RBsystem : public rclcpp::Node {
	public:
		// constructor for the rigid body system
		RBsystem(float freqsim, Quadcopter::dr_prop* dr, RBH::planes* planes, double h, double stiff, double damp)
		{
			brim_subscriber_ = this->create_subscription<commsmsgs::msg::Brimpub>("/GC/out/brim", 10, std::bind(&RBsystem::brim_callback, [this], std::placeholders::_1));
			prim_subscriber_ = this->create_subscription<commsmsgs::msg::Rrimpub>("/GC/out/prim", 10, std::bind(&RBsystem::prim_callback, [this], std::placeholders::_1));
			
			rbquadsim_publisher_ = this->create_publisher<commsmsgs::msg::Rbquadsimpub>("/GC/out/rbquadsim", 10);

			auto timer_callback = [this]() -> void {
				// what ever code to run every timer iteration
				this->RBstep();
			}

			initrb(freqsim, dr, planes, h, stiff, damp);

			// wait for a few ms for variables to finish initializing
			// I dont know for some reason we need a pause here
			for (int i = 0; i < 100; i++) {}

			start_time = clocky::now();
			loop_time = clocky::now();

			timer_pub_ = this->create_wall_timer(10ms, timer_callback);
		
		}
	private:
		// subscibers and publishers
		rclcpp::Subscription<commsmsgs::msg::Brimpub>::SharedPtr brim_subscriber_;
		rclcpp::Subscription<commsmsgs::msg::Rrimpub>::SharedPtr prim_subscriber_;
		rclcpp::Publisher<commsmsgs::msg::Rbquadsimpub>::SharedPtr rbquadsim_publisher_;

		rclcpp::TimerBase::SharedPtr timer_pub_;

		std::atomic<uint64_t> timestamp_;


		// constructor for the rigid body system
		void initrb(float freqsim, Quadcopter::dr_prop* dr, RBH::planes* planes, double h, double stiff, double damp);
		// functions to fetch and update the states
		void RBstep();

		// callback for the brim subscriber
		void brim_callback(const commsmsgs::msg::Brimpub::UniquePtr & msg);
		// callback for the prim subscriber
		void prim_callback(const commsmsgs::msg::Rrimpub::UniquePtr & msg);

		// fill the sparse matrices to avoid errors
		void fill_matrices();
		// complete a plane interaction calculation
		void plane_interaction(Eigen::SparseMatrix<double>* vel_imposed, Eigen::Vector3d* Int_for, Eigen::Matrix<double, 6, 1>* phins, Eigen::Matrix<double, 6, 1>* dphins, Eigen::Vector3d* Li, Eigen::SparseMatrix<double>* mat_An, Eigen::SparseMatrix<double>* mat_dot_An, Eigen::SparseMatrix<double>* mat_M, Eigen::SparseMatrix<double>* mat_inv_M, Eigen::SparseMatrix<double>* mat_dot_M, Eigen::SparseMatrix<double>* mat_star_M, Eigen::SparseMatrix<double>* mat_O, Eigen::SparseMatrix<double>* mat_star_O, Eigen::SparseMatrix<double>* mat_N, Eigen::SparseMatrix<double>* mat_dot_N, Eigen::SparseMatrix<double>* vel_body, RBH::planes* eqns_plane, Quadcopter::UAS* Udrone, double stiffness, double damping, double s_s, bool* contact, bool* pre_contact, bool* post_contact, bool* no_contact);
		// solve the plane interaction problem
		void interface_solution_CR(Eigen::Matrix <double, 6, 1>* nlambda, Eigen::SparseMatrix<double>* mat_M, Eigen::SparseMatrix<double>* mat_inv_M, Eigen::SparseMatrix<double>* mat_dot_M, Eigen::SparseMatrix<double>* mat_An, Eigen::SparseMatrix<double>* mat_dot_An, Eigen::SparseMatrix<double>* mat_O, Eigen::Matrix <double, 6, 1>* dphins, Eigen::Matrix <double, 6, 1>* phins, double s_s, Eigen::SparseMatrix<double>* V);

		// structs
		Quadcopter::UAS drone;
		RBH::planes plane_eqns;

		// Sparse Matrices
		Eigen::SparseMatrix <double> An_mat;
		Eigen::SparseMatrix <double> An_mat_dot;
		Eigen::SparseMatrix <double> M_mat;
		Eigen::SparseMatrix <double> M_mat_pre;
		Eigen::SparseMatrix <double> M_mat_dot;
		Eigen::SparseMatrix <double> M_mat_inv;
		Eigen::SparseMatrix <double> M_mat_star;
		Eigen::SparseMatrix <double> O_mat;
		Eigen::SparseMatrix <double> O_mat_star;
		Eigen::SparseMatrix <double> N_mat;
		Eigen::SparseMatrix <double> N_mat_dot;
		Eigen::SparseMatrix <double> imposed_vel;
		Eigen::SparseMatrix <double> global_vel;
		Eigen::SparseMatrix <double> q_dot;

		// Matrices
		Eigen::Vector3d Lambda_i;
		Eigen::Matrix <double, 6, 1> drphins;
		Eigen::Matrix <double, 6, 1> drdphins;
		Eigen::Vector3d dr_vel_lims;

		// sim properties
		double step_size;
		int sigd;
		int drplnum;

		// physical properties
		double stiffness;
		double damping;
		double rho;

		// interaction properties
		double drphin;
		double drdphin;

		// interaction flags
		bool contact;
		bool pre_contact;
		bool post_contact;
		bool no_contact;

		// helper vars for system setup
		// setup local variables
		// conditional variables
		bool mats_r;

		// numerical variables
		int count;
		int i;
		double freq;
		double pos_norm;

		// structs
		Quadcopter::draggrav dgout;

		// vectors
		Eigen::Vector3d DDP;
		Eigen::Vector3d Wvv;
		Eigen::Vector3d Fid;
		Eigen::Vector3d Int_for;
		Eigen::Matrix <double, 1, 6> droneCrvel;
		Eigen::Vector3d pos_diff;
		Eigen::MatrixXd velimp;
	};
}

#endif