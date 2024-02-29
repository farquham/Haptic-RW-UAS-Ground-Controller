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

namespace BMN {
	// a struct to store all the data that needs to be transferred between the different threads
	struct Var_Transfer {
		//general stuff
		//helper bools
		bool mats_ready;
		bool running;
		// bool contact_check;
		// bool pre_contact_check;
		// bool post_contact_check;
		// bool no_contact_check;

		// data logging variables
		// double fsim_real;
		// double fbrim_real;
		// double fbmn_real;
		// double fprim_real;
		// double fpx4_real;
		// Eigen::Vector2d brim_time;
		// Eigen::Vector2d prim_time;

		//RBsystem stuff
		//outputs
		Eigen::SparseMatrix <double> M_inv;
		Eigen::SparseMatrix <double> Ac;
		Eigen::SparseMatrix <double> vb;
		// Eigen::Vector3d Drag;
		// Eigen::Vector3d Gravity;
		// Eigen::Vector3d Interaction;
		// Eigen::Vector3d d_pos;
		// Eigen::Vector3d d_vel;
		// Eigen::Vector3d d_acc;
		// Eigen::Matrix3d d_RM;
		// //inputs
		// Eigen::Vector3d DDC;

		// //BRIM stuff
		// //out
		// Eigen::Vector3d BPDP;
		// //brim_phin_list
		// //brim_dot_phin_list

		// //in
		// //B_DDC
		// //bint_force_list

		// //BMN stuff
		// //out
		// Eigen::Matrix <double, 3, 1> bint_force_list;
		// Eigen::Vector3d B_DDC;
		// //in
		// Eigen::Matrix <double, 3, 1> brim_phin_list;
		// Eigen::Matrix <double, 3, 1> brim_dot_phin_list;

		// //PRIM stuff
		// //out
		// Eigen::Vector3d PPDP;

		// //in
		// //P_DDC
		// //d_pos

		// //PX4 stuff
		// //out
		// Eigen::Vector3d P_DDC;
		// Eigen::Vector3d P_DV;
		// Eigen::Vector3d P_DA;
		// //in
		// //PPDP

		// controller debugging
		Eigen::Vector3d vel_sp;
		Eigen::Vector3d vel_err;
		Eigen::Vector3d vel_int;
		Eigen::Vector3d vel_dot;
		Eigen::Vector3d acc_sp;

		// lim params
		Eigen::Vector3d BRIM_lims;
		Eigen::Vector3d BRIM_dlims;

		Eigen::Vector3d d_apos;
	};

	// struct to import the parameters
	struct setup_parameters {
		double k_b; // stiffness of the bubble
		double k_i; // stiffness of the interface
		double d_i; // damping of the interface
		double v_s;	// velocity scaling
		double p_s; // position scaling
		double b_r; // bubble radius
		double c_r; // contact radius
		double a_d; // activation distance
		double s_s; // step size
		double f_com;
		double v_l; // velocity limit
		double f_s; // force scaling
		Eigen::Vector3d flims; // force limits
		double phin_max; // max phin before adjusting vmax
		double vmaxchange; // max change in vmax
		double PSchange; // change spring constants in position region
		double VSchange; // change spring constants in velocity region
	};

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
	// setting up the inverse
	std::string inverse3_setup();
	// safety function to center the end effector
	void centerDevice(Haply::HardwareAPI::Devices::Inverse3 Inverse_object, Eigen::Vector3d* workspace_center, double* rest);
	// the main function for the bmn simulation which loops until the user stops it
	void BMNLoop(Haply::HardwareAPI::Devices::Inverse3 Inverse_object, Eigen::Vector3d* workspace_center, Var_Transfer* ptr, std::mutex* lock, setup_parameters* params);
	// runge kutta 4th order integrator for use throughout simulation
	void RK4_update(double* xn, double* xn_dot, double* h);
	void RK4_vec_update(Eigen::Vector3d* xn, Eigen::Vector3d xn_dot, double h);}

#endif