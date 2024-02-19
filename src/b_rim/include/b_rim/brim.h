#ifndef BRIM_H
#define BRIM_H

// include eigen dense and sparse and alse the quadcopter and haply stuff
#include <iostream>
#include <../../deps/Eigen3/Eigen/Dense>
#include <../../deps/Eigen3/Eigen/Sparse>
#include <../../deps/Eigen3/Eigen/QR>
// #include <Eigen/Dense>
// #include <Eigen/Sparse>
// #include <Eigen/QR>
#include <chrono>
#include <cstdio>
#include <mutex>
#include "../../deps/HardwareAPI/include/HardwareAPI.h"
#include "BubbleMethod.h"

namespace BRIM {
	class BRIM {
	public:
		// BRIM constructor
		BRIM(int frim);
		// BRIM loop starter
		void BRIMLoop(BMN::Var_Transfer* ptr, std::mutex* lock, int fcom1, int fcom2, int rim_type);
	private:
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

		// helper vars for system setup
		double frim;
		double h;
		double h_com1;
		double r_type;
		
		// runge kutta 4th order integrator for use throughout simulation
		void RK4_update(double* xn, double* xn_dot, double* h);
		void RK4_vec_update(Eigen::Vector3d* xn, Eigen::Vector3d xn_dot, double h);	
	};

}

#endif