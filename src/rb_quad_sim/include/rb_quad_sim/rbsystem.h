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
#include "commsmsgs/msg/guicontrols.hpp"

typedef std::chrono::high_resolution_clock clocky;
using namespace std::chrono_literals;

namespace RBsystem {		
	class RBsystem : public rclcpp::Node {
	public:
		// constructor for the rigid body system
		RBsystem() : Node("rb_quad_sim_node")
		{
			this->declare_parameter("freqsim", 100.0);
			this->declare_parameter("h", 0.0);
			this->declare_parameter("g_p", std::vector<float>{0.0, 0.0, 0.0});
			this->declare_parameter("l", 0.0);
			this->declare_parameter("w", 0.0);
			this->declare_parameter("he", 0.0);
			this->declare_parameter("In", std::vector<float>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
			this->declare_parameter("mass", 0.0);
			this->declare_parameter("con_rad", 0.0);
			this->declare_parameter("prop_dia", 0.0);
			this->declare_parameter("kprop", 0.0);
			this->declare_parameter("bprop", 0.0);
			this->declare_parameter("xypdis", 0.0);
			this->declare_parameter("zpdis", 0.0);
			this->declare_parameter("Tscale", 0);
			this->declare_parameter("PIDlims", std::vector<float>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
			this->declare_parameter("kp", std::vector<float>{0.0, 0.0, 0.0});
			this->declare_parameter("kvp", std::vector<float>{0.0, 0.0, 0.0});
			this->declare_parameter("kvi", std::vector<float>{0.0, 0.0, 0.0});
			this->declare_parameter("kvd", std::vector<float>{0.0, 0.0, 0.0});
			this->declare_parameter("kap", std::vector<float>{0.0, 0.0, 0.0});
			this->declare_parameter("krp", std::vector<float>{0.0, 0.0, 0.0});
			this->declare_parameter("kri", std::vector<float>{0.0, 0.0, 0.0});
			this->declare_parameter("krd", std::vector<float>{0.0, 0.0, 0.0});
			this->declare_parameter("krff", std::vector<float>{0.0, 0.0, 0.0});
			this->declare_parameter("vel_con_lims", std::vector<float>{0.0, 0.0});
			this->declare_parameter("n", std::vector<float>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
			this->declare_parameter("nhat", std::vector<float>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
			this->declare_parameter("poswall", 0.0);
			this->declare_parameter("negwall", 0.0);
			this->declare_parameter("d", std::vector<float>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
			this->declare_parameter("stiff", 0.0);
			this->declare_parameter("damp", 0.0);

			double freqsim = this->get_parameter("freqsim").as_double();
			double h = this->get_parameter("h").as_double();
			std::vector<double> g_p = this->get_parameter("g_p").as_double_array();
			double l = this->get_parameter("l").as_double();
			double w = this->get_parameter("w").as_double();
			double he = this->get_parameter("he").as_double();
			std::vector<double> In = this->get_parameter("In").as_double_array();
			double mass = this->get_parameter("mass").as_double();
			double con_rad = this->get_parameter("con_rad").as_double();
			double prop_dia = this->get_parameter("prop_dia").as_double();
			double kprop = this->get_parameter("kprop").as_double();
			double bprop = this->get_parameter("bprop").as_double();
			double xypdis = this->get_parameter("xypdis").as_double();
			double zpdis = this->get_parameter("zpdis").as_double();
			int Tscale = this->get_parameter("Tscale").as_int();
			std::vector<double> PIDlims = this->get_parameter("PIDlims").as_double_array();
			std::vector<double> kp = this->get_parameter("kp").as_double_array();
			std::vector<double> kvp = this->get_parameter("kvp").as_double_array();
			std::vector<double> kvi = this->get_parameter("kvi").as_double_array();
			std::vector<double> kvd = this->get_parameter("kvd").as_double_array();
			std::vector<double> kap = this->get_parameter("kap").as_double_array();
			std::vector<double> krp = this->get_parameter("krp").as_double_array();
			std::vector<double> kri = this->get_parameter("kri").as_double_array();
			std::vector<double> krd = this->get_parameter("krd").as_double_array();
			std::vector<double> krff = this->get_parameter("krff").as_double_array();
			std::vector<double> vel_con_lims = this->get_parameter("vel_con_lims").as_double_array();
			std::vector<double> n = this->get_parameter("n").as_double_array();
			std::vector<double> nhat = this->get_parameter("nhat").as_double_array();
			double poswall = this->get_parameter("poswall").as_double();
			double negwall = this->get_parameter("negwall").as_double();
			std::vector<double> d = this->get_parameter("d").as_double_array();
			double stiff = this->get_parameter("stiff").as_double();
			double damp = this->get_parameter("damp").as_double();

			brim_subscriber_ = this->create_subscription<commsmsgs::msg::Brimpub>("/GC/out/brim", 10, std::bind(&RBsystem::brim_callback, this, std::placeholders::_1));
			prim_subscriber_ = this->create_subscription<commsmsgs::msg::Rrimpub>("/GC/out/prim", 10, std::bind(&RBsystem::prim_callback, this, std::placeholders::_1));
			guicontrols_subscriber_ = this->create_subscription<commsmsgs::msg::Guicontrols>("/GC/internal/guictrls", 10, std::bind(&RBsystem::guicontrols_callback, this, std::placeholders::_1));
			
			rbquadsim_publisher_ = this->create_publisher<commsmsgs::msg::Rbquadsimpub>("/GC/out/rbquadsim", 10);

			// auto timer_callback = [this]() -> void {
			// 	// what ever code to run every timer iteration
			// 	this->RBstep();
			// }

			Quadcopter::dr_prop dr;
			dr.g_p = Eigen::Vector3d(g_p[0], g_p[1], g_p[2]);
			dr.l = l;
			dr.w = w;
			dr.h = h;
			dr.In(0,0) = In[0];
			dr.In(0,1) = In[1];
			dr.In(0,2) = In[2];
			dr.In(1,0) = In[3];
			dr.In(1,1) = In[4];
			dr.In(1,2) = In[5];
			dr.In(2,0) = In[6];
			dr.In(2,1) = In[7];
			dr.In(2,2) = In[8];
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
			planes.n(0,0) = n[0];
			planes.n(0,1) = n[1];
			planes.n(0,2) = n[2];
			planes.n(1,0) = n[3];
			planes.n(1,1) = n[4];
			planes.n(1,2) = n[5];
			planes.n(2,0) = n[6];
			planes.n(2,1) = n[7];
			planes.n(2,2) = n[8];
			planes.n(3,0) = n[9];
			planes.n(3,1) = n[10];
			planes.n(3,2) = n[11];
			planes.n(4,0) = n[12];
			planes.n(4,1) = n[13];
			planes.n(4,2) = n[14];
			planes.n(5,0) = n[15];
			planes.n(5,1) = n[16];
			planes.n(5,2) = n[17];
			planes.nhat(0,0) = nhat[0];
			planes.nhat(0,1) = nhat[1];
			planes.nhat(0,2) = nhat[2];
			planes.nhat(1,0) = nhat[3];
			planes.nhat(1,1) = nhat[4];
			planes.nhat(1,2) = nhat[5];
			planes.nhat(2,0) = nhat[6];
			planes.nhat(2,1) = nhat[7];
			planes.nhat(2,2) = nhat[8];
			planes.nhat(3,0) = nhat[9];
			planes.nhat(3,1) = nhat[10];
			planes.nhat(3,2) = nhat[11];
			planes.nhat(4,0) = nhat[12];
			planes.nhat(4,1) = nhat[13];
			planes.nhat(4,2) = nhat[14];
			planes.nhat(5,0) = nhat[15];
			planes.nhat(5,1) = nhat[16];
			planes.nhat(5,2) = nhat[17];
			planes.d = Eigen::Matrix <double, 6, 1>(d[0]*poswall, d[1]*negwall, d[2]*poswall, d[3]*negwall, d[4]*poswall, d[5]*poswall);

			initrb(freqsim, &dr, &planes, h, stiff, damp);

			// wait for a few ms for variables to finish initializing
			// I dont know for some reason we need a pause here
			for (int i = 0; i < 100; i++) {}

			start_time = clocky::now();

			auto time = 1000ms / freqsim;

			timer_pub_ = this->create_wall_timer(time, std::bind(&RBsystem::RBsystem::timer_callback, this));
		}
	private:
		void timer_callback() {
			// what ever code to run every timer iteration
			if (run) {
				this->RBstep();
			}
			
			// pushing outputs from the loop
			commsmsgs::msg::Rbquadsimpub msg{};
			msg.header.stamp = this->now();
			msg.running = run;
			msg.position.x = drone.get_state().g_pos[0];
			msg.position.y = drone.get_state().g_pos[1];
			msg.position.z = drone.get_state().g_pos[2];
			msg.velocity.x = drone.get_state().g_vel[0];
			msg.velocity.y = drone.get_state().g_vel[1];
			msg.velocity.z = drone.get_state().g_vel[2];
			msg.acceleration.x = drone.get_state().g_acc[0];
			msg.acceleration.y = drone.get_state().g_acc[1];
			msg.acceleration.z = drone.get_state().g_acc[2];
			msg.orientation.w = drone.get_state().g_att.w();
			msg.orientation.x = drone.get_state().g_att.x();
			msg.orientation.y = drone.get_state().g_att.y();
			msg.orientation.z = drone.get_state().g_att.z();
			msg.angular_velocity.x = drone.get_state().b_avel[0];
			msg.angular_velocity.y = drone.get_state().b_avel[1];
			msg.angular_velocity.z = drone.get_state().b_avel[2];
			// pose / orientation ???
			msg.drag.x = dgout.Fd[0];
			msg.drag.y = dgout.Fd[1];
			msg.drag.z = dgout.Fd[2];
			msg.gravity.x = dgout.Fg[0];
			msg.gravity.y = dgout.Fg[1];
			msg.gravity.z = dgout.Fg[2];
			msg.interaction.x = Int_for[0];
			msg.interaction.y = Int_for[1];
			msg.interaction.z = Int_for[2];
			msg.rotation_matrix.m11 = drone.get_state().r_mat(0, 0);
			msg.rotation_matrix.m12 = drone.get_state().r_mat(0, 1);
			msg.rotation_matrix.m13 = drone.get_state().r_mat(0, 2);
			msg.rotation_matrix.m21 = drone.get_state().r_mat(1, 0);
			msg.rotation_matrix.m22 = drone.get_state().r_mat(1, 1);
			msg.rotation_matrix.m23 = drone.get_state().r_mat(1, 2);
			msg.rotation_matrix.m31 = drone.get_state().r_mat(2, 0);
			msg.rotation_matrix.m32 = drone.get_state().r_mat(2, 1);
			msg.rotation_matrix.m33 = drone.get_state().r_mat(2, 2);

			RBH::matrix_to_msg(global_vel, msg.vg, this->get_logger());
			RBH::matrix_to_msg(An_mat, msg.ac, this->get_logger());
			RBH::matrix_to_msg(M_mat_inv, msg.m_inv, this->get_logger());


			msg.contact = contact;
			msg.pre_contact = pre_contact;
			msg.post_contact = post_contact;
			msg.no_contact = no_contact;
			msg.sim_freq = freq;

			//RCLCPP_INFO(this->get_logger(), "Publishing Simulated Drone Position: %f %f %f", msg.position.x, msg.position.y, msg.position.z);
			//RCLCPP_INFO(this->get_logger(), "Publishing Simulated Drone Velocity: %f %f %f", msg.velocity.x, msg.velocity.y, msg.velocity.z);
			//RCLCPP_INFO(this->get_logger(), "Publishing Simulated Drone Acceleration: %f %f %f", msg.acceleration.x, msg.acceleration.y, msg.acceleration.z);
			//RCLCPP_INFO(this->get_logger(), "Publishing Simulated Drone Orientation: %f %f %f %f", msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
			
			// publish the message
			rbquadsim_publisher_->publish(msg);
		}
		// subscibers and publishers
		rclcpp::Subscription<commsmsgs::msg::Brimpub>::SharedPtr brim_subscriber_;
		rclcpp::Subscription<commsmsgs::msg::Rrimpub>::SharedPtr prim_subscriber_;
		rclcpp::Publisher<commsmsgs::msg::Rbquadsimpub>::SharedPtr rbquadsim_publisher_;
		rclcpp::Subscription<commsmsgs::msg::Guicontrols>::SharedPtr guicontrols_subscriber_;

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
		// callback for the guicontrols subscriber
		void guicontrols_callback(const commsmsgs::msg::Guicontrols::UniquePtr & msg);

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
		double fsim;

		bool run;

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

		std::chrono::_V2::system_clock::time_point start_time;
		std::chrono::duration<double> loop_time;
	};
}

int main(int argc, char * argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RBsystem::RBsystem>());
	rclcpp::shutdown();
	return 0;
}

#endif