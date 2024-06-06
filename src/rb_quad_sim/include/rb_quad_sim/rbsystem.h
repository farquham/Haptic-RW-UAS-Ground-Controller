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
		RBsystem(float freqsim, double h, float g_p[3], float l, float w, float he, float In[3][3], float mass, float con_rad, float prop_dia, float kprop, float bprop, float xypdis, float zpdis, int Tscale, float PIDlims[13], float kp[3], float kvp[3], float kvi[3], float kvd[3], float kap[3], float krp[3], float kri[3], float krd[3], float krff[3], float vel_con_lims[2], float n[6][3], float nhat[6][3], float poswall, float negwall, float d[6], float stiff, float damp) : Node("rb_quad_sim_node")
		{
			brim_subscriber_ = this->create_subscription<commsmsgs::msg::Brimpub>("/GC/out/brim", 10, std::bind(&RBsystem::brim_callback, [this], std::placeholders::_1));
			prim_subscriber_ = this->create_subscription<commsmsgs::msg::Rrimpub>("/GC/out/prim", 10, std::bind(&RBsystem::prim_callback, [this], std::placeholders::_1));
			
			rbquadsim_publisher_ = this->create_publisher<commsmsgs::msg::Rbquadsimpub>("/GC/out/rbquadsim", 10);

			auto timer_callback = [this]() -> void {
				// what ever code to run every timer iteration
				this->RBstep();
			}

			Quadcopter::dr_prop dr;
			dr.g_p = Eigen::Vector3d(g_p[0], g_p[1], g_p[2]);
			dr.l = l;
			dr.w = w;
			dr.h = h;
			dr.In = Eigen::Matrix3d(In[0][0], In[0][1], In[0][2], In[1][0], In[1][1], In[1][2], In[2][0], In[2][1], In[2][2]);
			dr.mass = mass;
			dr.con_rad = con_rad;
			dr.prop_dia = prop_dia;
			dr.k = kprop/(1.2*(std::pow(prop_dia,4))); 
			dr.b = bprop/(1.2*(std::pow(prop_dia,5))); 
			dr.xypdis = xypdis;
			dr.zpdis = zpdis;
			dr.Tscale = Tscale;
			dr.PIDlims = Eigen::Matrix<double, 13, 1>(PIDlims[0]* (3.14159/180), PIDlims[1]* (3.14159/180), PIDlims[2]* (3.14159/180), PIDlims[3], PIDlims[4], PIDlims[5], PIDlims[6], PIDlims[7], PIDlims[8], PIDlims[9], PIDlims[10], PIDlims[11], PIDlims[12]);
			dr.kp = Eigen::Vector3d(kp[0], kp[1], kp[2]);
			dr.kvp = Eigen::Vector3d(kvp[0], kvp[1], kvp[2]);
			dr.kvi = Eigen::Vector3d(kvi[0], kvi[1], kvi[2]);
			dr.kvd = Eigen::Vector3d(kvd[0], kvd[1], kvd[2]);
			dr.kap = Eigen::Vector3d(kap[0], kap[1], kap[2]);
			dr.krp = Eigen::Vector3d(krp[0], krp[1], krp[2]);
			dr.kri = Eigen::Vector3d(kri[0], kri[1], kri[2]);
			dr.krd = Eigen::Vector3d(krd[0], krd[1], krd[2]);
			dr.krff = Eigen::Vector3d(krff[0], krff[1], krff[2]);
			dr.vel_con_lims = Eigen::Vector2d(vel_con_lims[0], vel_con_lims[1]);
			RBH::planes planes;
			planes.n = Eigen::Matrix <double, 6, 3>(n[0][0], n[0][1], n[0][2], n[1][0], n[1][1], n[1][2], n[2][0], n[2][1], n[2][2], n[3][0], n[3][1], n[3][2], n[4][0], n[4][1], n[4][2], n[5][0], n[5][1], n[5][2]);
			planes.nhat = Eigen::Matrix <double, 6, 3>(nhat[0][0], nhat[0][1], nhat[0][2], nhat[1][0], nhat[1][1], nhat[1][2], nhat[2][0], nhat[2][1], nhat[2][2], nhat[3][0], nhat[3][1], nhat[3][2], nhat[4][0], nhat[4][1], nhat[4][2], nhat[5][0], nhat[5][1], nhat[5][2]);
			planes.d = Eigen::Matrix <double, 6, 1>(d[0]*poswall, d[1]*negwall, d[2]*poswall, d[3]*negwall, d[4]*poswall, d[5]*poswall);

			initrb(freqsim, &dr, &planes, h, stiff, damp);

			// wait for a few ms for variables to finish initializing
			// I dont know for some reason we need a pause here
			for (int i = 0; i < 100; i++) {}

			start_time = clocky::now();
			loop_time = clocky::now();

			time = 1000ms / freqsim;

			timer_pub_ = this->create_wall_timer(time, timer_callback);
		
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
	int main(int argc, char * argv[]) {
		rclcpp::init(argc, argv);
		rclcpp::spin(std::make_shared<RBsystem>());
		rclcpp::shutdown();
		return 0;
	}
}

#endif