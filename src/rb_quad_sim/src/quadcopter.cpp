#include "Quadcopter.h"


//default constructor
Quadcopter::UAS::UAS() {
	// global frame and body frame lienar state vectors
	global_pos = {0.0,0.0,1.0};
	global_vel.setZero();
	global_acc.setZero();
	body_vel.setZero();
	body_acc.setZero();
	// global and body frame angular state vectors
	global_apos.setZero();
	global_dapos.setZero();
	global_att.setIdentity();
	body_avel.setZero();
	global_avel.setZero();
	body_aacc.setZero();
	// matrix info
	rotors_speed.setZero();
	rot_mat.setIdentity();
	jac_mat.setIdentity();

	// general UAS properties
	// physical properties
	L = 0.0;
	W = 0.0;
	H = 0.0;
	I.setZero();
	I_inv = I.inverse();
	m = 0.0;
	rho = 1.2;
	con_rad = 0.0;
	p_dia = 0.0;
	// internal dynamics parameters
	k = 1.0e-04;
	b = 4.0e-06;
	xypdis = 0;
	zpdis = 0;
	// sim properties
	count = 0;
	con_count_pv = 0;
	con_count_att = 0;
	step_size = 0.01;
	sigd = 6;

	// controller parameters
	con_params.kp_pos.setZero();
	con_params.kp_vel.setZero();
	con_params.ki_vel.setZero();
	con_params.kd_vel.setZero();
	con_params.kp_att.setZero();
	con_params.kp_rate.setZero();
	con_params.ki_rate.setZero();
	con_params.kd_rate.setZero();
	con_params.ff_rate.setZero();

	con_params.att_lims.setZero();
	con_params.rate_lims.setZero();
	con_params.lim_vel_horz = 0.0;
	con_params.lim_vel_up = 0.0;
	con_params.lim_vel_down = 0.0;
	con_params.lim_thr_max = 0.0;
	con_params.lim_thr_min = 0.0;
	con_params.lim_tilt = 0.0;
	con_params.lim_thr_xy_margin = 0.0;

	vel_con_lims = {0.0, 0.0};

	des_att_sp.pitch_body = 0.0;
	des_att_sp.roll_body = 0.0;
	des_att_sp.yaw_body = 0.0;
	des_att_sp.yaw_sp_move_rate = 0.0;
	des_att_sp.thrust_body = {0.0,0.0,0.0};
	des_att_sp.reset_integral = false;
	des_att_sp.q_d = {0.0,0.0,0.0,0.0};
	start_att.q_d = {0.0,0.0,0.0,0.0};

	// desired moments and forces outputs
	des_F.setZero();
	des_M.setZero();
	// setpoint vectors
	vel_sp.setZero();
	vel_int.setZero();
	acc_sp.setZero();
	thr_sp.setZero();
	rate_sp.setZero();
	tor_sp.setZero();
	tor_actual.setZero();
	yaw_sp = 0.0;
	yawspeed_sp = 0.0;

	vel_dot.setZero();
	_vel_dot.setZero();
	_vel.setZero();

	// vi helper vars
	vi.setOnes();
	vi = vi/1;
	vi_old.setOnes();
	vi_old = vi_old/2;
	vi_s.setOnes();
	vi_s = vi_s/1;
	vi_s_old.setOnes();
	vi_s_old = vi_s_old/2;

	con = false;
	pre_con = false;
	post_con = false;
	no_con = false;
	recovered = false;

	contact_count = 0;
	pre_contact_count = 0;

	desiredDronePos = global_pos;

	// set the initial attitude to be the identity quaternion to ensure the drone is flat at the start
	Eigen::Vector3d start_z = {0,0,1};
	double tempy = 0.0;
	QH::bodyzToAttitude(start_z, &tempy, &start_att);
	global_att = start_att.q_d;
	QH::canonical(&global_att);
}

// constructor that initialzes the quadcopter object with the given parameters
Quadcopter::UAS::UAS(dr_prop* props, double h) {
	// global frame and body frame lienar state vectors
	global_pos = props->g_p;
	global_vel.setZero();
	global_acc.setZero();
	body_vel.setZero();
	body_acc.setZero();
	// global and body frame angular state vectors
	global_apos.setZero();
	global_dapos.setZero();
	global_att.setIdentity();
	body_avel.setZero();
	global_avel.setZero();
	body_aacc.setZero();
	// matrix info
	rotors_speed.setZero();
	rot_mat.setIdentity();
	jac_mat.setIdentity();

	// general UAS properties
	// physical properties
	L = props->l;
	W = props->w;
	H = props->h;
	I = props->In;
	I_inv = I.inverse();
	m = props->mass;
	rho = 1.2;
	con_rad = props->con_rad;
	p_dia = props->prop_dia;
	// internal dynamics parameters
	k = props->k;
	b = props->b;
	xypdis = props->xypdis;
	zpdis = props->zpdis;
	// sim properties
	count = 0;
	con_count_pv = 0;
	con_count_att = 0;
	step_size = h;
	sigd = 6;

	// controller parameters
	con_params.kp_pos = {props->kp[0], props->kp[1], props->kp[2]};
	con_params.kp_vel = {props->kvp[0], props->kvp[1], props->kvp[2]};
	con_params.ki_vel = {props->kvi[0], props->kvi[1], props->kvi[2]};
	con_params.kd_vel = {props->kvd[0], props->kvd[1], props->kvd[2]};
	con_params.kp_att = {props->kap[0], props->kap[1], props->kap[2]};
	con_params.kp_rate = {props->krp[0], props->krp[1], props->krp[2]};
	con_params.ki_rate = {props->kri[0], props->kri[1], props->kri[2]};
	con_params.kd_rate = {props->krd[0], props->krd[1], props->krd[2]};
	con_params.ff_rate = {props->krff[0], props->krff[1], props->krff[2]};

	con_params.att_lims = {props->PIDlims[0],props->PIDlims[1],props->PIDlims[2]};
	con_params.rate_lims = {props->PIDlims[3],props->PIDlims[4],props->PIDlims[5]};
	con_params.lim_vel_horz = props->PIDlims[6];
	con_params.lim_vel_up = props->PIDlims[7];
	con_params.lim_vel_down = props->PIDlims[8];
	con_params.lim_thr_min = props->PIDlims[9];
	con_params.lim_thr_max = props->PIDlims[10];
	con_params.lim_tilt = props->PIDlims[11];
	con_params.lim_thr_xy_margin = props->PIDlims[12];

	vel_con_lims = props->vel_con_lims;

	des_att_sp.pitch_body = 0.0;
	des_att_sp.roll_body = 0.0;
	des_att_sp.yaw_body = 0.0;
	des_att_sp.yaw_sp_move_rate = 0.0;
	des_att_sp.thrust_body = {0.0,0.0,0.0};
	des_att_sp.reset_integral = false;
	des_att_sp.q_d = {0.0,0.0,0.0,0.0};
	start_att.q_d = {0.0,0.0,0.0,0.0};

	// desired moments and forces outputs
	des_F.setZero();
	des_M.setZero();
	// setpoint vectors
	vel_sp.setZero();
	vel_int.setZero();
	acc_sp.setZero();
	thr_sp.setZero();
	rate_sp.setZero();
	tor_sp.setZero();
	tor_actual.setZero();
	yaw_sp = 0.0;
	yawspeed_sp = 0.0;

	vel_dot.setZero();
	_vel_dot.setZero();
	_vel.setZero();

	// vi helper vars
	vi.setOnes();
	vi = vi/1;
	vi_old.setOnes();
	vi_old = vi_old/2;
	vi_s.setOnes();
	vi_s = vi_s/1;
	vi_s_old.setOnes();
	vi_s_old = vi_s_old/2;

	con = false;
	pre_con = false;
	post_con = false;
	no_con = false;
	recovered = false;

	contact_count = 0;
	pre_contact_count = 0;

	desiredDronePos = global_pos;

	// set the initial attitude to be the identity quaternion to ensure the drone is flat at the start
	Eigen::Vector3d start_z = {0,0,1};
	double tempy = 0.0;
	QH::bodyzToAttitude(start_z, &tempy, &start_att);
	global_att = start_att.q_d;
	QH::canonical(&global_att);
}

// Public Function Definitions
// sets the DDP after impacts
void Quadcopter::UAS::set_DDP() {
	desiredDronePos = global_pos;
}

// returns the positions of the obstacle
Eigen::Vector3d Quadcopter::UAS::get_pos() {
	return global_pos;
}

// returns the current state of the obstacle
Quadcopter::state Quadcopter::UAS::get_state() {
	Quadcopter::state out;
	// global and body frame vectors
	// linear
	out.g_pos = global_pos;
	out.g_vel = global_vel;
	out.g_acc = global_acc;
	out.b_vel = body_vel;
	out.b_acc = body_acc;
	// angular
	out.g_apos = global_apos;
	out.g_att = global_att;
	out.g_dapos = global_dapos;
	out.g_datt = global_datt;
	out.b_avel = body_avel;
	out.g_avel = global_avel;
	out.b_aacc = body_aacc;
	// rotation and jacobian matrices
	out.r_mat = rot_mat;
	out.j_mat = jac_mat;
	// physical properties
	out.L = L;
	out.W = W;
	out.H = H;
	out.I = I;
	out.mass = m;
	out.con_rad = con_rad;

	// debugging controller calcs
	out.vel_sp = vel_sp;
	out.vel_err = _vel_dot;
	out.vel_int = vel_int;
	out.vel_dot = vel_dot;
	out.acc_sp = acc_sp;

	return out;
}

// Controlls the drone using the px4 casecade PID controller
// the different frequencies help with maintaining stability in the drone
void Quadcopter::UAS::Controller(Eigen::Vector3d* DDP, bool* contact, bool* pre_contact, bool* post_contact, bool* no_contact){
	con = *contact;
	pre_con = *pre_contact;
	post_con = *post_contact;
	no_con = *no_contact;

	double dis = (global_pos - *DDP).norm();
	if (dis <= 0.4) {
		recovered = true;
	}
	else {
		recovered = false;
	}

	// only runs the position and velocity controllers at 50 Hz
	if (((step_size*con_count_pv) >= (1/50))) {
		if (recovered) {
			desiredDronePos = (*DDP);
		}
		QH::Pos_Controller(&desiredDronePos, &vel_sp, &global_pos, &con_params);
		vel_dot = global_acc;
		QH::Vel_Controller(&vel_int, &vel_dot, &acc_sp, &thr_sp, &con_params, &_vel_dot, &_vel, &global_vel, &step_size, &vel_sp, m, &no_con, &post_con, &vel_con_lims);
		QH::Acc_to_Att(&thr_sp, &yaw_sp, &des_att_sp, &yawspeed_sp, &des_F, &rot_mat);
		con_count_pv = 0;
	}
	// runs the attitude controller at 100 Hz
	if ((step_size*con_count_att) >= (1/100)) {
		QH::Att_Controller(&des_att_sp, &rate_sp, &global_att, &con_params, &yawspeed_sp);
		con_count_att = 0;
	}

	// runs the rate controller at full simulation speed
	QH::Rate_Controller(&tor_sp, &tor_actual, &rate_sp, &body_avel, &body_aacc, &con_params, &rate_int, &step_size, &des_M);
	con_count_pv += 1;
	con_count_att += 1;
}

// calculates the required rotor speeds to achieve the desired forces and torques
void Quadcopter::UAS::Mixer() {
	// desired moments and forces from controller
	Eigen::Vector3d Moment = des_M;
	Eigen::Vector3d Force = des_F;

	//std::cout << "Desired Body F: " << Force[0] << ", " << Force[1] << ", " << Force[2] << std::endl;
	//std::cout << "Desired Body M: " << Moment[0] << ", " << Moment[1] << ", " << Moment[2] << std::endl;

	// moment arms used to calculate the required rotor thrusts
	Eigen::Vector3d rB1 = { -1 * xypdis, -1 * xypdis , -1 * zpdis };
	Eigen::Vector3d rB2 = { 1 * xypdis, 1 * xypdis , -1 * zpdis };
	Eigen::Vector3d rB3 = { 1 * xypdis, -1 * xypdis , -1 * zpdis };
	Eigen::Vector3d rB4 = { -1 * xypdis, 1 * xypdis , -1 * zpdis };

	Eigen::Matrix <double, 4, 3> rBs;
	rBs << rB1[0], rB1[1], rB1[2],
		rB2[0], rB2[1], rB2[2],
		rB3[0], rB3[1], rB3[2],
		rB4[0], rB4[1], rB4[2];

	// simplified thrust and torque coefficients
	double kk = k * rho * (std::pow(p_dia,4));
	double bb = b * rho * (std::pow(p_dia,5));

	// calculate the breakdown in the thrust and moments into thrust, and all 3 moments, by rotor
	Eigen::Vector4d zThrust = {(Force[2]/4), (Force[2]/4), (Force[2]/4), (Force[2]/4)};
	Eigen::Vector4d xMoment = { (Moment[0] / (4 * rB1[1])), (Moment[0] / (4 * rB2[1])), (Moment[0] / (4 * rB3[1])), (Moment[0] / (4 * rB4[1])) };
	Eigen::Vector4d yMoment = { (Moment[1] / (4 * rB1[0])), (Moment[1] / (4 * rB2[0])), (Moment[1] / (4 * rB3[0])), (Moment[1] / (4 * rB4[0])) };
	Eigen::Vector4d zMomentrpm = { -(Moment[2] / (4 * bb)), -(Moment[2] / (4 * bb)), (Moment[2] / (4 * bb)), (Moment[2] / (4 * bb)) };

	// combine all the moment and thrusts that originate from rotor thrust
	Eigen::Vector4d Thrust_tot = xMoment + yMoment + zThrust;

	// adapt it for non hover flight
	QH::adaptedreqThrusts(&Thrust_tot, rho, p_dia, &global_vel, &body_avel, &rBs, &rot_mat, &vi_s, &vi_s_old);
	Thrust_tot = Thrust_tot/kk;
	// add in the affect of the moments that orginate from rotor torque
	Eigen::Vector4d Tots = Thrust_tot + zMomentrpm;

	// desired rotor speeds calculations
	double x1 = sqrt(abs(Tots[0]));
	double x2 = sqrt(abs(Tots[1]));
	double x3 = sqrt(abs(Tots[2]));
	double x4 = sqrt(abs(Tots[3]));

	// saving the calculated rotor speeds
	rotors_speed[0] = copysignf(x1, Tots[0]);
	rotors_speed[1] = copysignf(x2, Tots[1]);
	rotors_speed[2] = copysignf(x3, Tots[2]);
	rotors_speed[3] = copysignf(x4, Tots[3]);
}

// calculates the wind force acting on the object for a given wind speed vector
Eigen::Vector3d Quadcopter::UAS::wind_force(Eigen::Vector3d* Wvv) {
	// first transform it into the body frame
	Eigen::Vector3d Bwvv;
	Bwvv = rot_mat.transpose() * (*Wvv);
	double Dx, Dy, Dz;
	// treats the obstacle as a cuboid for simplicity and calculates the drag force
	Dx = (rho / 2) * (Bwvv[0] * Bwvv[0]) * 0.4 * ((3 * (L * H) + 2 * (L * W)) + (pow((L * H * W), (2 / 3))));
	Dy = (rho / 2) * (Bwvv[1] * Bwvv[1]) * 0.4 * ((3 * (L * H) + 2 * (L * W)) + (pow((L * H * W), (2 / 3))));
	Dz = (rho / 2) * (Bwvv[2] * Bwvv[2]) * 1 * ((4 * (L * H) + 1 * (L * W)) + (pow((L * H * W), (2 / 3))));
	// outputs the result
	Eigen::Vector3d output;
	output[0] = Dx;
	output[1] = Dy;
	output[2] = Dz;
	return output;
}

// calculates the linear and angular accelerations of the drone; including both internal and external forces
Quadcopter::draggrav Quadcopter::UAS::Accelerator(Eigen::Vector3d* Fid) {
	// moment arms used to calculate moments caused by rotor thrusts
	Eigen::Vector3d rB1 = { -1 * xypdis, -1 * xypdis , -1 * zpdis };
	Eigen::Vector3d rB2 = { 1 * xypdis, 1 * xypdis , -1 * zpdis };
	Eigen::Vector3d rB3 = { 1 * xypdis, -1 * xypdis , -1 * zpdis };
	Eigen::Vector3d rB4 = { -1 * xypdis, 1 * xypdis , -1 * zpdis };

	Eigen::Matrix <double, 4, 3> rBs;
	rBs << rB1[0], rB1[1], rB1[2],
		rB2[0], rB2[1], rB2[2],
		rB3[0], rB3[1], rB3[2],
		rB4[0], rB4[1], rB4[2];

	// calculate the non hover thrusts produced by the rotors
	Eigen::Vector4d T = { 0,0,0,0 };
	QH::adaptedThrusts(&T, rho, k, p_dia, &rotors_speed, &global_vel, &body_avel, &rBs, &rot_mat, &vi, &vi_old);

	// internal moment calculation from the 4 rotors
	Eigen::Vector4d Q = { 0,0,0,0 };
	Q[0] = (rotors_speed[0] * rotors_speed[0]) * b * rho * (std::pow(p_dia,5));
	Q[1] = (rotors_speed[1] * rotors_speed[1]) * b * rho * (std::pow(p_dia,5));
	Q[2] = (rotors_speed[2] * rotors_speed[2]) * b * rho * (std::pow(p_dia,5));
	Q[3] = (rotors_speed[3] * rotors_speed[3]) * b * rho * (std::pow(p_dia,5));
	
	// drag force calculation
	double Dx, Dy, Dz;
	Dx = (rho / 2) * (body_vel[0] * body_vel[0]) * 0.4 * ((3 * (L * H) + 2 * (L * W)) + (pow((L * H * W), (2 / 3))));
	Dy = (rho / 2) * (body_vel[1] * body_vel[1]) * 0.4 * ((3 * (L * H) + 2 * (L * W)) + (pow((L * H * W), (2 / 3))));
	Dz = (rho / 2) * (body_vel[2] * body_vel[2]) * 1 * ((4 * (L * H) + 1 * (L * W)) + (pow((L * H * W), (2 / 3))));

	draggrav dg;
	dg.Fd[0] = Dx;
	dg.Fd[1] = Dy;
	dg.Fd[2] = Dz;

	Eigen::Vector3d bDrag = { Dx, Dy, Dz };
	Eigen::Vector3d gDrag = rot_mat * bDrag;

	// gravity force calulation
	Eigen::Vector3d Fg = { 0,0, -1 * (m * 9.81) };

	dg.Fg = Fg;

	// add together the body frame forces (drag, and thrusts)
	Eigen::Vector3d body_forces = { -1 * copysignf(Dx, body_vel[0]), -1 * copysignf(Dy, body_vel[1]), T[0]+T[1]+T[2]+T[3]-copysignf(Dz, body_vel[2]) };
	// add together the global forces (gravity, and applied forces)
	Eigen::Vector3d global_forces = (*Fid) + Fg;
	// add the two force vectors together in the global frame
	Eigen::Vector3d F = global_forces + (rot_mat * body_forces);

	//std::cout << "Body F no grav: " << body_forces[0] << ", " << body_forces[1] << ", " << body_forces[2] << std::endl;
	//std::cout << "Global F: " << F[0] << ", " << F[1] << ", " << F[2] << std::endl;

	dg.F_internal = F;

	// moments caused by thrust force calulation
	Eigen::Vector3d Thrust1 = { 0 ,0,T[0] };
	Eigen::Vector3d Thrust2 = { 0 ,0,T[1] };
	Eigen::Vector3d Thrust3 = { 0 ,0,T[2] };
	Eigen::Vector3d Thrust4 = { 0 ,0,T[3] };
	Eigen::Vector3d Mt1 = rB1.cross(Thrust1);
	Eigen::Vector3d Mt2 = rB2.cross(Thrust2);
	Eigen::Vector3d Mt3 = rB3.cross(Thrust3);
	Eigen::Vector3d Mt4 = rB4.cross(Thrust4);
	
	// add together all the moments in the body frame
	Eigen::Vector3d Mt = Mt1 + Mt2 + Mt3 + Mt4;
	Eigen::Vector3d M = { Mt[0], -1*Mt[1], (Mt[2] - Q[0] - Q[1] + Q[2] + Q[3]) };
	tor_actual = M;

	//std::cout << "Body M: " << M[0] << ", " << M[1] << ", " << M[2] << std::endl;

	// linear acceleration calculation in the global frame
	Eigen::Vector3d g_acc = (F) / m;
	// convert the global acc back to the body frame
	body_acc = rot_mat.transpose() * g_acc;

	// angular acceleration calculation in the body frame
	Eigen::Vector3d temp = I * body_avel;
	Eigen::Vector3d Mhat = body_avel.cross(temp);
	Eigen::Vector3d Mdiff = M - Mhat;
	body_aacc = I_inv * Mdiff;


	count += 1;

	return dg;
}

// updates the state of the obstacle when using kinematic constraints
void Quadcopter::UAS::update_state() {
	// update the body frame state vectors
	QH::RK4_vec_update(&body_vel, body_acc, step_size);
	QH::RK4_vec_update(&body_avel, body_aacc, step_size);
	
	// update the global frame state vectors
	global_acc = rot_mat * body_acc;
	global_avel = rot_mat * body_avel;
	QH::RK4_vec_update(&global_vel, global_acc, step_size);
	QH::RK4_vec_update(&global_pos, global_vel, step_size);

	//std::cout << "global_acc: " << global_acc[0] << ", " << global_acc[1] << ", " << global_acc[2] << std::endl;
	//std::cout << "global_vel: " << global_vel[0] << ", " << global_vel[1] << ", " << global_vel[2] << std::endl;
	//std::cout << "global_pos: " << global_pos[0] << ", " << global_pos[1] << ", " << global_pos[2] << std::endl;
	
	// update the global_att quaternion using body_avel
	Eigen::Vector3d local_avel = body_avel;
	Eigen::Matrix<double, 3, 4> G;
	Eigen::Vector4d qvec = { global_att.w(), global_att.x(), global_att.y(), global_att.z() };
	G << -qvec[1], qvec[0], qvec[3], -qvec[2],
		 -qvec[2], -qvec[3], qvec[0], qvec[1],
		 -qvec[3], qvec[2], -qvec[1], qvec[0];
	Eigen::Matrix4d IdentityM = Eigen::Matrix4d::Identity();
	Eigen::Vector4d qdot = 0.5 * G.transpose() * local_avel;
	double qw = qvec[0];
	QH::RK4_update(&qw, &qdot[0], &step_size);
	Eigen::Vector3d qv = { qvec[1], qvec[2], qvec[3] };
	Eigen::Vector3d qvdot = {qdot[1], qdot[2], qdot[3]};
	QH::RK4_vec_update(&qv, qvdot, step_size);

	// output the updated quaternion
	global_att.w() = qw;
	global_att.x() = qv[0];
	global_att.y() = qv[1];
	global_att.z() = qv[2];
	global_att.normalize();

	// update the global_datt quaternion
	global_datt.w() = qdot[0];
	global_datt.x() = qdot[1];
	global_datt.y() = qdot[2];
	global_datt.z() = qdot[3];

	//std::cout << "body_avel: " << body_avel[0] << ", " << body_avel[1] << ", " << body_avel[2] << std::endl;
	//std::cout << "global_att: " << global_att.w() << ", " << global_att.x() << ", " << global_att.y() << ", " << global_att.z() << std::endl;
	//std::cout << "global_datt: " << global_datt.w() << ", " << global_datt.x() << ", " << global_datt.y() << ", " << global_datt.z() << std::endl;

	// still calculate global_dapos and global_apos for use in RBsystem calcs !!!
	Eigen::Vector3d temp_apos = global_apos;
	QH::ToEulerAngles(&global_att ,&global_apos);
	global_dapos = (global_apos - temp_apos) / step_size;

	//std::cout << "Euler Angles: " << global_apos[0] << ", " << global_apos[1] << ", " << global_apos[2] << std::endl;
	
	// update the rotation and jacobian matrices
	rot_mat = global_att.toRotationMatrix();
	QH::JacobianMatrix(&global_apos, &jac_mat);
}

// updates the state of the obstacle when using constitutive relations
void Quadcopter::UAS::CR_update(Eigen::Matrix <double, 1, 6>* imposed_vel) {
	// update the body frame state vectors
	global_vel[0] = (*imposed_vel)[0];
	global_vel[1] = (*imposed_vel)[1];
	global_vel[2] = (*imposed_vel)[2];
	
	//global_pos = global_pos + global_vel * step_size;
	QH::RK4_vec_update(&global_pos, global_vel, step_size);

	// update the global_att quaternion
	// N/A until friction contact is added
	
	// update the rotation and jacobian matrices
	rot_mat = global_att.toRotationMatrix();

	body_vel = rot_mat.transpose() * global_vel;
}