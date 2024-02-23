#ifndef Q_helpers_H
#define Q_helpers_H

// only really need eigen for the matrix math
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace QH{
    // struct to hold all the attitude setpoint info
	struct Veh_Att_sp {
		double roll_body;
		double pitch_body;
		double yaw_body;

		double yaw_sp_move_rate;

		Eigen::Quaterniond q_d;

		Eigen::Vector3d thrust_body;

		bool reset_integral;
	};

	// struct to hold all PX4 PID parameters needed for the drone controller
	struct PID_params {
		Eigen::Vector3d kp_pos;
		Eigen::Vector3d kp_vel;
		Eigen::Vector3d ki_vel;
		Eigen::Vector3d kd_vel;
		Eigen::Vector3d kp_att;
		Eigen::Vector3d kp_rate;
		Eigen::Vector3d ki_rate;
		Eigen::Vector3d kd_rate;
		Eigen::Vector3d ff_rate;

		Eigen::Vector3d att_lims;
		Eigen::Vector3d rate_lims;
		double lim_vel_horz;
		double lim_vel_up;
		double lim_vel_down;
		double lim_thr_min;
		double lim_thr_max;
		double lim_tilt;
		double lim_thr_xy_margin;
	};
	
	// Function Initializations
	// controller internal functions
	// position controller function
	void Pos_Controller(Eigen::Vector3d* DDP, Eigen::Vector3d* vel_sp, Eigen::Vector3d* global_pos, PID_params* con_params);
	// velocity controller function
	void Vel_Controller(Eigen::Vector3d* vel_int, Eigen::Vector3d* vel_dot, Eigen::Vector3d* acc_sp, Eigen::Vector3d* thr_sp, PID_params* con_params, Eigen::Vector3d* _vel_dot, Eigen::Vector3d* _vel, Eigen::Vector3d* global_vel, double* step_size, Eigen::Vector3d* vel_sp, double m, bool* no_contact, bool* post_contact, Eigen::Vector2d* con_lims);
	// acceleration controller function / velocity controller helper
	void Acc_Controller(Eigen::Vector3d* acc_sp, Eigen::Vector3d* thr_sp, PID_params* con_params, double m);
	// acceleration to atttitude helper function
	void Acc_to_Att(Eigen::Vector3d* thr_sp, double* yaw_sp, Veh_Att_sp* des_att_sp, double* yawspeed_sp, Eigen::Vector3d* des_F, Eigen::Matrix3d* RM);
	// attitude controller function
	void Att_Controller(QH::Veh_Att_sp* des_att_sp, Eigen::Vector3d* rate_sp, Eigen::Quaterniond* global_att, PID_params* con_params, double* yawspeed_sp);
	// rate controller function
	void Rate_Controller(Eigen::Vector3d* tor_sp, Eigen::Vector3d* tor_actual, Eigen::Vector3d* rate_sp, Eigen::Vector3d* body_avel, Eigen::Vector3d* body_aacc, PID_params* con_params, Eigen::Vector3d* rate_int, double* step_size, Eigen::Vector3d* des_M);
	
	// controller helper functions
	// helper to calculate vel cot from vels and limit if needed
	void veldt_calc(Eigen::Vector3d* vel_dot, Eigen::Vector3d* _vel_dot, Eigen::Vector3d* _vel, Eigen::Vector3d* global_vel, double* step_size);
	// heper to limit the tilt of the drone to ensure that the propellers are always pointing up
	void limitTilt(Eigen::Vector3d* body_u, Eigen::Vector3d world_u, double* lim);
	// helper to calculate the required quaternion attitude from the desired thrust vector
	void bodyzToAttitude(Eigen::Vector3d body_z, double* yaw_sp, Veh_Att_sp* att_sp);
	// dcm_z for quaternions
	Eigen::Vector3d dcm_z(Eigen::Quaterniond* q);
	// helper to convert a quaternion to its canonical form
	void canonical(Eigen::Quaterniond* original);
	// helper to convert a quaternion to euler angles
	void ToEulerAngles(Eigen::Quaterniond *q, Eigen::Vector3d *euler);
	// helper to find jacobian matrix of euler angles
	void JacobianMatrix(Eigen::Vector3d* global_apos, Eigen::Matrix3d* jac_mat);
	// attitude helper to find quaternion from two vectors
	Eigen::Quaterniond Quat_twovec(Eigen::Vector3d* v1, Eigen::Vector3d* v2);
	// helper to constrain the x and y velocities to a maximum value
	Eigen::Vector2d ConstrainXY(Eigen::Vector2d sp_,  Eigen::Vector2d sp, double lim);
	// helper to return the sign of a value
	int signfinder(double value);

	// runge kutta 4th order integrator for use throughout simulation
	void RK4_update(double* xn, double* xn_dot, double* h);
	void RK4_vec_update(Eigen::Vector3d* xn, Eigen::Vector3d xn_dot, double h);

	// thrust helper functions
	// Newton Raphton method to find the induced velocity
	void inducedVelocity(double* vi, double* vh, double* vz, double* vxy);
	// induced velocity polynomial function
	double vi_poly(double* vi, double* vh, double* vz, double* vxy);
	// induced velocity polynomial function derivative
	double vi_poly_d(double* vi, double* vz, double* vxy);
	// same but for req thrusts
	// Newton Raphton method to find the induced velocity for required thrusts
	void vhinducedVelocity(double* vi , double* vi_pre, Eigen::Vector3d* vinf, double thrust, double rho, double Ar);
	// induced velocity polynomial function for required thrusts
	double vi_poly_2(double* vi, Eigen::Vector3d* vinf, double thrust, double rho, double Ar);
	// hover velocity polynomial function for required thrusts
	void vh_poly(double* vh, double* vi, double* vz, double* vxy);
	// thrust helper
	// function to help calculate how non hover flight change the thrust produced
	void adaptedThrusts(Eigen::Vector4d* thrusts, double rho, double Ct, double prop_dia, Eigen::Vector4d* rotors_speed, Eigen::Vector3d* global_vel, Eigen::Vector3d* body_avel, Eigen::Matrix <double, 4, 3>* r_arms, Eigen::Matrix3d* rot_mat, Eigen::Vector4d* v_i, Eigen::Vector4d* v_i_old);
	// function to help calculate how non hover flight changes the thrust required
	void adaptedreqThrusts(Eigen::Vector4d* thrusts, double rho, double prop_dia, Eigen::Vector3d* global_vel, Eigen::Vector3d* body_avel, Eigen::Matrix <double, 4, 3>* r_arms, Eigen::Matrix3d* rot_mat, Eigen::Vector4d* v_i_s, Eigen::Vector4d* v_i_s_old);
}


#endif