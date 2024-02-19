#ifndef Quadcopter_H
#define Quadcopter_H

#include <iostream>
#include <../../deps/Eigen3/Eigen/Dense>
#include <../../deps/Eigen3/Eigen/Geometry>
// #include <Eigen/Dense>
// #include <Eigen/Geometry>
#include "BubbleMethod.h"
#include "Q_helpers.h"

namespace Quadcopter {
	// struct to export the drones drag and gravity data
	struct draggrav {
		Eigen::Vector3d Fd;
		Eigen::Vector3d Fg;
		Eigen::Vector3d F_internal;
	};

	// struct to export the state after the controller has been updated
	struct state {
		// global frame and body frame lienar state vectors
		Eigen::Vector3d g_pos;
		Eigen::Vector3d g_vel;
		Eigen::Vector3d g_acc;
		Eigen::Vector3d b_vel;
		Eigen::Vector3d b_acc;
		// global and body frame angular state vectors
		Eigen::Vector3d g_apos;
		Eigen::Quaterniond g_att;
		Eigen::Vector3d g_dapos;
		Eigen::Quaterniond g_datt;
		Eigen::Vector3d b_avel;
		Eigen::Vector3d g_avel;
		Eigen::Vector3d b_aacc;
		// object properties
		Eigen::Matrix3d r_mat;
		Eigen::Matrix3d j_mat;
		// Eigen::Matrix3d j_mat;
		double L;
		double W;
		double H;
		Eigen::Matrix3d I;
		double mass;
		double con_rad;

		// controller debugging helpers
		Eigen::Vector3d vel_sp;
		Eigen::Vector3d vel_err;
		Eigen::Vector3d vel_int;
		Eigen::Vector3d vel_dot;
		Eigen::Vector3d acc_sp;
	};

	// struct for defining the state of a quadcopter for setup purposes
	struct dr_prop {
		Eigen::Vector3d g_p;
		double l;
		double w;
		double h;
		Eigen::Matrix3d In;
		double mass;
		double con_rad;
		double prop_dia;
		double k;
		double b;
		double xypdis;
		double zpdis;
		int Tscale;
		Eigen::Matrix<double, 13, 1> PIDlims;
		Eigen::Vector3d kp;
		Eigen::Vector3d kvp;
		Eigen::Vector3d kvi;
		Eigen::Vector3d kvd;
		Eigen::Vector3d kap;
		Eigen::Vector3d krp;
		Eigen::Vector3d kri;
		Eigen::Vector3d krd;
		Eigen::Vector3d krff;

		Eigen::Vector2d vel_con_lims;
	};
	
	class UAS {
	public:
		// drone constructors
		UAS(void);
		UAS(dr_prop* props, double h);
		// state editing functions
		// set the DDP for the controller to current SDP
		// for use after impact only
		void set_DDP();
		// state fetching functions
		Eigen::Vector3d get_pos();
		state get_state();
		// update the drone control loop
		void Controller(Eigen::Vector3d* DDP, bool* contact, bool* pre_contact, bool* post_contact, bool* no_contact);
		// update the mixer loop to find the required rotor speeds
		void Mixer();
		// helper to calculate a wind force for a given wind velocity vector
		Eigen::Vector3d wind_force(Eigen::Vector3d* Wvv);
		// update the simulation state
		draggrav Accelerator(Eigen::Vector3d* Fid);
		// update the drone internally
		void update_state();
		// update the drone externally
		void CR_update(Eigen::Matrix <double, 1, 6>* imposed_vel);
	private:
		// sim state info
		// global frame and body frame lienar state vectors
		Eigen::Vector3d global_pos;
		Eigen::Vector3d global_vel;
		Eigen::Vector3d global_acc;
		Eigen::Vector3d body_vel;
		Eigen::Vector3d body_acc;
		// global and body frame angular state vectors
		Eigen::Vector3d global_apos;
		Eigen::Quaterniond global_att;
		Eigen::Vector3d global_dapos;
		Eigen::Quaterniond global_datt;
		Eigen::Vector3d body_avel;
		Eigen::Vector3d global_avel;
		Eigen::Vector3d body_aacc;
		// matrix info
		Eigen::Vector4d rotors_speed;
		Eigen::Matrix3d rot_mat;
		Eigen::Matrix3d jac_mat;
		
		// physical properties
		double L;
		double W;
		double H;
		Eigen::Matrix3d I;
		Eigen::Matrix3d I_inv;
		double m;
		double rho;
		double con_rad;
		double p_dia;
		// internal dynamics parameters
		double k;
		double b;
		double xypdis;
		double zpdis;
		// sim properties
		int count;
		int con_count_pv;
		int con_count_att;
		double step_size;
		int sigd;
		

		// controller parameters
		QH::PID_params con_params;
		QH::Veh_Att_sp des_att_sp;
		QH::Veh_Att_sp start_att;
		// desired moments and forces outputs
		Eigen::Vector3d des_F;
		Eigen::Vector3d des_M;
		// setpoint vectors
		Eigen::Vector3d vel_sp;
		Eigen::Vector3d vel_int;
		Eigen::Vector3d acc_sp;
		Eigen::Vector3d thr_sp;
		Eigen::Vector3d rate_sp;
		Eigen::Vector3d rate_int;
		Eigen::Vector3d tor_sp;
		Eigen::Vector3d tor_actual;
		double yaw_sp;
		double yawspeed_sp;

		// helper vars for proper acc calcs
		Eigen::Vector3d vel_dot;
		Eigen::Vector3d _vel_dot;
		Eigen::Vector3d _vel;

		// helper vars for proper vi calcs
		Eigen::Vector4d vi;
		Eigen::Vector4d vi_old;
		Eigen::Vector4d vi_s;
		Eigen::Vector4d vi_s_old;

		// helper vars for reseting
		bool con;
		bool pre_con;
		bool post_con;
		bool no_con;
		bool recovered;
		int contact_count;
		int pre_contact_count;

		Eigen::Vector3d desiredDronePos;

		Eigen::Vector2d vel_con_lims;
	};
}

#endif