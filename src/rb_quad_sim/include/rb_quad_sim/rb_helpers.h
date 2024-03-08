#ifndef RB_helpers_H
#define RB_helpers_H

// include eigen dense and sparse and alse the quadcopter class
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include <chrono>
#include <cstdio>
#include <mutex>
#include "quadcopter.h"

// include the ros2 stuff
#include <std_msgs/msg/float64_multi_array.hpp>

namespace RBH {
	// struct for defining the state of a plane
	struct planes {
		Eigen::Matrix <double, 6, 3> n;
		Eigen::Matrix <double, 6, 3> nhat;
		Eigen::Matrix <double, 6, 1> d;
	};
		
	// helper to find the interface distance
    void plane_interface_distance(Eigen::Vector3d* object_pos, Eigen::Matrix <double, 6, 1>* phin_list, planes* plane_eqns, double object_con_rad);
	// helper to find the interface velocity
    void plane_interface_velocity(Eigen::SparseMatrix<double>* An_mat, Eigen::SparseMatrix<double>* global_vels, Eigen::Matrix <double, 6, 1>* dphin_list);
	// helper to find the interface jacobian
    void plane_interface_jacobian(Eigen::SparseMatrix<double>* An_mat, planes* plane_eqns);
	// matrix functions
	// mass matrix stuff
    void massMatrix(Eigen::SparseMatrix<double>* M_mat,  Eigen::SparseMatrix<double>* M_mat_star, Eigen::SparseMatrix<double>* M_mat_inv, Eigen::SparseMatrix<double>* M_mat_dot, Eigen::SparseMatrix<double>* M_mat_pre, Eigen::SparseMatrix<double>* N, Quadcopter::UAS* drone, double step_size);
    // star mass matrix stuff
	void SmassMatrix(Eigen::SparseMatrix<double>* M_mat_star, Quadcopter::UAS* drone);
	// star mass matrix work
	void Smasswork(Eigen::Matrix4d* Iars, Quadcopter::UAS* drone);
    // coriolis matrix stuff
    void omegaMatrix(Eigen::SparseMatrix<double>* O_mat, Eigen::SparseMatrix<double>* O_mat_star, Eigen::SparseMatrix<double>* N_mat, Eigen::SparseMatrix<double>* N_mat_dot, Eigen::SparseMatrix<double>* M_mat_star, Quadcopter::UAS* drone);
    // star coriolis matrix stuff
	void SomegaMatrix(Eigen::SparseMatrix<double>* O_mat_star, Quadcopter::UAS* drone);
	// star coriolis matrix work
	Eigen::Matrix4d Somegawork(Eigen::Quaterniond* qdot, Eigen::Matrix3d* I, Eigen::Quaterniond* q);
	// transformation matrix
    void transformMatrix(Eigen::SparseMatrix<double>* N_mat, Quadcopter::UAS* drone);
	// transformation matrix time derivative
    void dotTransform(Eigen::SparseMatrix<double>* N_mat_dot, Quadcopter::UAS* drone);
	// time derivative matrices
    void dotJacobian(Eigen::SparseMatrix<double>* An_mat_dot, planes* plane_eqns);
	// extra helpers
	void sparse_fill(Eigen::SparseMatrix<double>* tb_filled, Eigen::Matrix3d* t_fill, int row, int col);
	void sparse_replace(Eigen::SparseMatrix<double>* tb_replaced, Eigen::Matrix3d* t_replace, int row, int col);
	void sparse_fill4(Eigen::SparseMatrix<double>* tb_filled, Eigen::Matrix4d* t_fill, int row, int col);
	void sparse_replace4(Eigen::SparseMatrix<double>* tb_replaced, Eigen::Matrix4d* t_replace, int row, int col);

	void matrix_to_msg(Eigen::SparseMatrix<double>* min, std_msgs::msg::Float64MultiArray* mout);
}

#endif