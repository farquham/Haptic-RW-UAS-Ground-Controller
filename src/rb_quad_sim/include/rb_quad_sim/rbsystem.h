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

namespace RBsystem {		
	class RBsystem {
	public:
		// constructor for the rigid body system
		RBsystem(Quadcopter::dr_prop* dr, RBH::planes* planes, double h, double stiff, double damp);
		// functions to fetch and update the states
		void RBLoop(BMN::Var_Transfer* ustate, std::mutex* lock, double fcom);

	private:
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
	};
}

#endif