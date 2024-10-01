#include "../include/rb_quad_sim/quad_helpers.h"

// controller helpers
// Position P controller
void QH::Pos_Controller(Eigen::Vector3d* DDP, Eigen::Vector3d* vel_sp, Eigen::Vector3d* global_pos, PID_params* con_params) {
	// complete controller step
	// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DDP: %f, %f, %f", (*DDP)[0], (*DDP)[1], (*DDP)[2]);
	// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "global_pos: %f, %f, %f", (*global_pos)[0], (*global_pos)[1], (*global_pos)[2]);
	Eigen::Vector3d vel_sp_pos = (*DDP - *global_pos).cwiseProduct(con_params->kp_pos);
	// account for feedforward term
	*vel_sp = vel_sp_pos;
	// constrain velocities to ensure they remain within physcial limits
	// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "vel_sp: %f, %f, %f", (*vel_sp)[0], (*vel_sp)[1], (*vel_sp)[2]);
	Eigen::Vector2d vel_sp_xy = QH::ConstrainXY(vel_sp_pos.block(0,0,2,1), (*vel_sp-vel_sp_pos).block(0,0,2,1), con_params->lim_vel_horz);
	double vel_sp_z = ((*vel_sp)[2] < -1*con_params->lim_vel_down) ? -1*con_params->lim_vel_down : (((*vel_sp)[2] > con_params->lim_vel_up) ? con_params->lim_vel_up : (*vel_sp)[2]);
	*vel_sp = {vel_sp_xy[0], vel_sp_xy[1], vel_sp_z};
	// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "vel_sp final: %f, %f, %f", (*vel_sp)[0], (*vel_sp)[1], (*vel_sp)[2]);
}

// Velocity PID controller
void QH::Vel_Controller(Eigen::Vector3d* vel_int, Eigen::Vector3d* vel_dot, Eigen::Vector3d* acc_sp, Eigen::Vector3d* thr_sp, PID_params* con_params, Eigen::Vector3d* _vel_dot, Eigen::Vector3d* _vel, Eigen::Vector3d* global_vel, double* step_size, Eigen::Vector3d* vel_sp, double m, bool* no_contact, bool* post_contact, Eigen::Vector2d* con_lims) {
	// constrain vel int in vertical direction
	(*vel_int)[2] = ((*vel_int)[2] < -9.81) ? -9.81 : (((*vel_int)[2] > 9.81) ? 9.81 : (*vel_int)[2]);

	// low pass filter acceleration term to remove any sudden changes (if abs(jerk) >= 9.81)
	//veldt_calc(vel_dot, _vel_dot, _vel, global_vel, step_size);

	// complete controller step
	// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "vel sp: %f, %f, %f", (*vel_sp)[0], (*vel_sp)[1], (*vel_sp)[2]);
	// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "global_vel: %f, %f, %f", (*global_vel)[0], (*global_vel)[1], (*global_vel)[2]);
	Eigen::Vector3d vel_err = *vel_sp - *global_vel;
	Eigen::Vector3d acc_sp_vel = vel_err.cwiseProduct(con_params->kp_vel) + (*vel_int).cwiseProduct(con_params->ki_vel) - (*vel_dot).cwiseProduct(con_params->kd_vel);
	// account for feedforward term
	(*acc_sp)[0] = acc_sp_vel[0];
	(*acc_sp)[1] = acc_sp_vel[1];
	(*acc_sp)[2] += acc_sp_vel[2];

	// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "vel err: %f, %f, %f", vel_err[0], vel_err[1], vel_err[2]);
	// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "vel int: %f, %f, %f", (*vel_int)[0], (*vel_int)[1], (*vel_int)[2]);
	// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "vel dot: %f, %f, %f", (*vel_dot)[0], (*vel_dot)[1], (*vel_dot)[2]);
	// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "acc sp: %f, %f, %f", (*acc_sp)[0], (*acc_sp)[1], (*acc_sp)[2]);

	(*_vel_dot) = vel_err;

	// acceleration conrol equations
	Acc_Controller(acc_sp, thr_sp, con_params, m);

	// vertical int anti windup
	if (((*thr_sp)[2] <= con_params->lim_thr_min && vel_err[2] >= 0.f) ||
		((*thr_sp)[2] >= con_params->lim_thr_max && vel_err[2] <= 0.f)) {
		(*vel_int)[2] = 0.f;
	}

	// prioritize vertical control while keeping a horz margin
	Eigen::Vector2d thr_sp_xy = { (*thr_sp)[0], (*thr_sp)[1] };
	double thr_sp_xy_norm = thr_sp_xy.norm();
	double thr_max_sqrd = con_params->lim_thr_max * con_params->lim_thr_max;

	// determine how much vertical thrust remains after accounting for horz margin
	double allocated_horz_thr = std::min(thr_sp_xy_norm, con_params->lim_thr_xy_margin);
	double thr_z_max_sqrd = thr_max_sqrd - allocated_horz_thr * allocated_horz_thr;

	// saturate max vert thr
	(*thr_sp)[2] = std::min((*thr_sp)[2], sqrt(thr_z_max_sqrd));

	// quantify remaining horz thrust
	double thr_xy_max_sqrd = thr_max_sqrd - (*thr_sp)[2] * (*thr_sp)[2];
	double thr_max_xy = 0.0;

	if (thr_xy_max_sqrd > 0.f) {
		thr_max_xy = sqrt(thr_xy_max_sqrd);
	}

	// saturate max horz thrust
	if (thr_sp_xy_norm > thr_max_xy) {
		thr_sp_xy = thr_sp_xy / thr_sp_xy_norm * thr_max_xy;
		(*thr_sp)[0] = thr_sp_xy[0];
		(*thr_sp)[1] = thr_sp_xy[1];
	}

	//if (vel_err.norm() < 1.0) {
		// use ARW to help reduce errors caused by integrator saturation
		Eigen::Vector2d acc_sp_xy_prod = { (*thr_sp)[0], (*thr_sp)[1]};
		acc_sp_xy_prod = acc_sp_xy_prod * (1/(m));
		double arw_gain = 2.f/con_params->kp_vel[0];

		// only run ARW loop if signal is saturated
		Eigen::Vector2d acc_sp_xy = (*acc_sp).block(0,0,2,1);
		Eigen::Vector2d acc_limd_xy = (acc_sp_xy.squaredNorm() > acc_sp_xy_prod.squaredNorm())
								  	? acc_sp_xy_prod : acc_sp_xy;
		Eigen::Vector2d vel_err_xy_temp = vel_err.block(0,0,2,1) - ((acc_sp_xy - acc_limd_xy) * arw_gain);
		vel_err[0] = vel_err_xy_temp[0];
		vel_err[1] = vel_err_xy_temp[1];

		// update integral
		*vel_int += vel_err * *step_size;
	//}

	// reset the vel_int if the drone is about to contact a plane (has not actually contacted yet)
	// if using the actual contact it will be slightly delayed letting some bad values through
	// if ((std::abs(((*vel_int).norm())) > 1.0) && (std::abs(vel_err.norm()) < 1.0)) {
	// 	std::cout << "No contact: " << (*no_contact) << std::endl;
	// 	std::cout << "vel err: " << vel_err[0] << ", " << vel_err[1] << ", " << vel_err[2] << " Norm: " << std::abs(vel_err.norm()) << std::endl;
	// 	std::cout << "vel int: " << (*vel_int)[0] << ", " << (*vel_int)[1] << ", " << (*vel_int)[2] << " Norm: " << std::abs((*vel_int).norm()) << std::endl;
	// }

	if ((*no_contact) && (std::abs(((*vel_int).norm())) > (*con_lims)[0]) && (std::abs(vel_err.norm()) < (*con_lims)[1])) {
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Resetting vel_int");
		*vel_int = {0, 0, 0};
	}

	// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "thr sp: %f, %f, %f", (*thr_sp)[0], (*thr_sp)[1], (*thr_sp)[2]);
}

// Acceleration "controller" helper for vel control
void QH::Acc_Controller(Eigen::Vector3d* acc_sp, Eigen::Vector3d* thr_sp, PID_params* con_params, double m){
	// generate the desired body frame
	Eigen::Vector3d body_z = {(*acc_sp)[0], (*acc_sp)[1], 9.81};
	body_z.normalize();
	// limit the tilt angle of the drone
	Eigen::Vector3d world_z = {0,0,1};
	QH::limitTilt(&body_z, world_z, &con_params->lim_tilt);
	// scale thrust assuming hover thrust produces standard gravity
	double acc_sp_z = (*acc_sp)[2];
	double coll_thrust = m * (acc_sp_z + 9.81);
	// limit thrust to min allowed
	coll_thrust = std::max(coll_thrust, con_params->lim_thr_min);
	coll_thrust = std::min(coll_thrust, con_params->lim_thr_max);
	// project thrsut to planned body attitude
	*thr_sp = body_z * coll_thrust;
}

// Acceleration/thrust to Attitude set point conversion
void QH::Acc_to_Att(Eigen::Vector3d* thr_sp, double* yaw_sp, Veh_Att_sp* des_att_sp, double* yawspeed_sp, Eigen::Vector3d* des_F, Eigen::Matrix3d* RM){
	// convert desired thrust vector to desired attitude quaternion
	bodyzToAttitude(*thr_sp, yaw_sp, des_att_sp);
	// output the results
	des_att_sp->thrust_body[2] = (*thr_sp).norm();
	des_att_sp->yaw_sp_move_rate = *yawspeed_sp;
	*des_F = des_att_sp->thrust_body;
}

// Attitude PID controller
void QH::Att_Controller(QH::Veh_Att_sp* des_att_sp, Eigen::Vector3d* rate_sp, Eigen::Quaterniond* global_att, PID_params* con_params, double* yawspeed_sp) {
	Eigen::Quaterniond qd_local = des_att_sp->q_d;

	// reduced attitude calculation to priortize roll and pitch
	Eigen::Vector3d e_z = QH::dcm_z(global_att);
	Eigen::Vector3d e_z_d = QH::dcm_z(&qd_local);

	Eigen::Quaterniond qd_red = QH::Quat_twovec(&e_z, &e_z_d);

	// in the corner case where the vehicle and thrust input have completely opposite direction
	// in that case qd_red is equal to qd_local
	if (std::fabs(qd_red.x()) > (1.f - 1e-5f) || std::fabs(qd_red.y()) > (1.f - 1e-5f)){
		qd_red = qd_local;
	}
	else {
		// transform rotation from current to desired thrust vector into a world frame reduced desired attitude
		qd_red *= (*global_att);
	}

	// mix the full and reduced desired attitudes
	Eigen::Quaterniond qd_mix = qd_red.inverse() * qd_local;
	QH::canonical(&qd_mix);
	qd_mix.w() = (qd_mix.w() < -1.f) ? -1.f: ((qd_mix.w() > 1.f) ? 1.f : qd_mix.w());
	qd_mix.z() = (qd_mix.z() < -1.f) ? -1.f: ((qd_mix.z() > 1.f) ? 1.f : qd_mix.z());
	qd_local = qd_red * Eigen::Quaterniond(cosf(0.4 * acosf(qd_mix.w())), 0, 0, sin(0.4 * asinf(qd_mix.z())));

	//std::cout << "quat sp: " << qd_local.w() << ", " << qd_local.x() << ", " << qd_local.y() << ", " << qd_local.z() << std::endl;
	//std::cout << "global att: " << (*global_att).w() << ", " << (*global_att).x() << ", " << (*global_att).y() << ", " << (*global_att).z() << std::endl;

	// find the error quaternion
	Eigen::Quaterniond qe = (*global_att).inverse() * qd_local;

	// calculate the rate setpoint
	QH::canonical(&qe);

	Eigen::Vector3d qeimag = {qe.x(), qe.y(), qe.z()};
	Eigen::Vector3d eq = 2.f * qeimag;
	Eigen::Vector3d rate_setpoint = eq.cwiseProduct(con_params->kp_att);

	// feed forward yaw setpoint rate
	if (std::isfinite(*yawspeed_sp)) {
		rate_setpoint += (*global_att).inverse().toRotationMatrix().col(2) * *yawspeed_sp;
	}

	qd_mix.w() = (qd_mix.w() < -1.f) ? -1.f: ((qd_mix.w() > 1.f) ? 1.f : qd_mix.w());

	Eigen::Vector3d temp_lims = con_params->att_lims;

	// limit the rate output
	for (int i = 0; i < 3; i++) {
		if (rate_setpoint(i) < -1 * temp_lims(i)) {
			rate_setpoint(i) = -1 * temp_lims(i);
		}
		else if (rate_setpoint(i) > temp_lims(i)) {
			rate_setpoint(i) = temp_lims(i);
		} else {
			rate_setpoint(i) = rate_setpoint(i);
		}
	}

	*rate_sp = rate_setpoint;
}

// Rate PID controller
void QH::Rate_Controller(Eigen::Vector3d* tor_sp, Eigen::Vector3d* tor_actual, Eigen::Vector3d* rate_sp, Eigen::Vector3d* body_avel, Eigen::Vector3d* body_aacc, PID_params* con_params, Eigen::Vector3d* rate_int, double* step_size, Eigen::Vector3d* des_M) {
	// determine if torque is saturated or not
	Eigen::Vector3d unallocated_torque = *tor_sp - *tor_actual;
	Eigen::Matrix <bool, 3, 1> sat_pos = {false, false, false};
	Eigen::Matrix <bool, 3, 1> sat_neg = {false, false, false};
	for (int i = 0; i < 3; i++) {
		if (unallocated_torque(i) > __FLT_EPSILON__){
			sat_pos(i) = true;
		}
		else if (unallocated_torque(i) < -__FLT_EPSILON__){
			sat_neg(i) = true;
		}
	}

	//std::cout << "rate sp: " << (*rate_sp)[0] << ", " << (*rate_sp)[1] << ", " << (*rate_sp)[2] << std::endl;
	//std::cout << "body_avel: " << (*body_avel)[0] << ", " << (*body_avel)[1] << ", " << (*body_avel)[2] << std::endl;
	// calculate the rate error
	Eigen::Vector3d rate_err = *rate_sp - *body_avel;
	double cur_rate_err = 0.0;

	// update the integral
	for (int i = 0; i < 3; i++) {
		cur_rate_err = rate_err(i);
		// if (sat_pos(i)) {
		// 	rate_err(i) = std::min(cur_rate_err, 0.0);
		// }
		// if (sat_neg(i)) {
		// 	rate_err(i) = std::max(cur_rate_err, 0.0);
		// }
		// I term factor: reduce the I gain with increasing rate error.
		// This counteracts a non-linear effect where the integral builds up quickly upon a large setpoint
		// change (noticeable in a bounce-back effect after a flip).
		// The formula leads to a gradual decrease w/o steps, while only affecting the cases where it should:
		// with the parameter set to 400 degrees, up to 100 deg rate error, i_factor is almost 1 (having no effect),
		// and up to 200 deg error leads to <25% reduction of I.
		double i_factor = rate_err(i) / ((400 * 2 * 3.14159)/360);
		double max_check = 1.0f - i_factor * i_factor;
		i_factor = std::max(0.0, max_check);

		// perform intergration using FOH
		double rate_i = (*rate_int)(i) + i_factor * con_params->ki_rate(i) * rate_err(i) * *step_size;

		if (std::isfinite(rate_i)) {
			(*rate_int)(i) = (rate_i < -con_params->rate_lims(i)) ? -con_params->rate_lims(i) : ((rate_i > con_params->rate_lims(i)) ? con_params->rate_lims(i) : rate_i);
		}
	}

	// calculate the desired torque
	*tor_sp = rate_err.cwiseProduct(con_params->kp_rate) + (*rate_int) - (*body_aacc).cwiseProduct(con_params->kd_rate) + (*rate_sp).cwiseProduct(con_params->ff_rate);
	*des_M = *tor_sp;

	//std::cout << "tor sp: " << (*tor_sp)[0] << ", " << (*tor_sp)[1] << ", " << (*tor_sp)[2] << std::endl;
}

// helper function to calculate vel_dot and use a LPF using the jerk to evaluate whether its a good value or not
void QH::veldt_calc(Eigen::Vector3d* vel_dot, Eigen::Vector3d* _vel_dot, Eigen::Vector3d* _vel, Eigen::Vector3d* global_vel, double* step_size){
	// find the acceleration
	*vel_dot = (*global_vel - *_vel)/(*step_size);
	// find the jerk
	//Eigen::Vector3d acc_dot = (*vel_dot - *_vel_dot)/(*step_size);

	// low pass filter the acceleration using
	if ((*vel_dot).norm() >= 10.0) {
		(*vel_dot).setZero();
	}

	*_vel = *global_vel;
	//*_vel_dot = *vel_dot;
}

// limits the tilt angle of the drone
void QH::limitTilt(Eigen::Vector3d* body_u, Eigen::Vector3d world_u, double* lim) {
	// determine current tilt

	double dprod_unit = (*body_u).dot(world_u);
	double angle = std::acos(dprod_unit);
	// limit tilt
	angle = std::min(angle, *lim);
	Eigen::Vector3d reject = *body_u - (dprod_unit * world_u);

	// when the two vectors are already parallel
	if (reject.squaredNorm() < __FLT_EPSILON__) {
		reject[0] = 1.0;
	}

	//output the result after normalizing
	*body_u = cosf(angle) * world_u + sinf(angle) * reject.normalized();
}

// converts the desired body frame thrust vector into the desired attitude
void QH::bodyzToAttitude(Eigen::Vector3d bodyz, double* yaw_sp, QH::Veh_Att_sp* att_sp){
	// if body_z is zero vector or no dir set to safe value
	if (bodyz.squaredNorm() < __FLT_EPSILON__){
		bodyz[2] = 1.f;
	}

	bodyz.normalize();

	//std::cout << "bodyz: " << bodyz[0] << ", " << bodyz[1] << ", " << bodyz[2] << std::endl;

	// vector of desired yaw direction in XY plane, rotated by the yaw input
	Eigen::Vector3d y_c_dir = {-1*sinf(*yaw_sp), -1*cosf(*yaw_sp), 0.f};

	// find desired bodyx axis which is orthogonal to bodyz
	Eigen::Vector3d bodyx = bodyz.cross(y_c_dir);

	// keep front pointing in correct direction even while inverted
	if (bodyz[2] < 0.f){
		bodyx = -bodyx;
	}

	if (std::abs(bodyz[2]) < 0.000001f){
		// desired thrust is in XY plane, set X downside to construct correct matrix, but yaw component will not be used actually
		bodyx.setZero();
		bodyx[2] = 1.f;
	}

	bodyx.normalize();

	//std::cout << "bodyx: " << bodyx[0] << ", " << bodyx[1] << ", " << bodyx[2] << std::endl;

	// desired bodyy axis
	Eigen::Vector3d bodyy = bodyz.cross(bodyx);
	bodyy.normalize();

	//std::cout << "bodyy: " << bodyy[0] << ", " << bodyy[1] << ", " << bodyy[2] << std::endl;

	Eigen::Matrix3d R_sp;
	R_sp.setZero();
	// fill rotation matrix
	for (int i = 0; i < 3; i++){
		R_sp(i,0) = bodyx[i];
		R_sp(i,1) = bodyy[i];
		R_sp(i,2) = bodyz[i];
	}

	// copy rotation matrix to quaternion setpoint
	Eigen::Quaterniond q_sp(R_sp);
	att_sp->q_d = q_sp;

	// record keeping only
	Eigen::Vector3d euler_sp = R_sp.eulerAngles(0,1,2);
	att_sp->roll_body = euler_sp[0];
	att_sp->pitch_body = euler_sp[1];
	att_sp->yaw_body = euler_sp[2];
}

// quaternion helper
Eigen::Vector3d QH::dcm_z(Eigen::Quaterniond* q) {
	Eigen::Vector3d R_z;
	// find the Z axis of a rotation matrix for a given quaternion using below equations
	R_z[0] = 2 * ((*q).w() * (*q).y() + (*q).x() * (*q).z());
	R_z[1] = 2 * ((*q).y() * (*q).z() - (*q).w() * (*q).x());
	R_z[2] = (*q).w() * (*q).w() - (*q).x() * (*q).x() - (*q).y() * (*q).y() + (*q).z() * (*q).z();
	return R_z;
}

// helper function to calculate the canonicalized version of a quaternion
void QH::canonical(Eigen::Quaterniond* original) {
	// find OG quaternion in vector form
	Eigen::Vector4d og = { (*original).w(), (*original).x(), (*original).y(), (*original).z() };
	Eigen::Vector4d output = og;
	for (int i = 0; i < 4; i++) {
		// if the value is not zero
		if (std::fabs(og(i)) > __FLT_EPSILON__) {
			// multiply the entire vector by this sign
			output =  (og * signfinder(og(i)));
			// return result in quaternion form
			*original = Eigen::Quaterniond(output(0), output(1), output(2), output(3));
			return;
		}
	}
	// if non are non zero than just return og
	*original = Eigen::Quaterniond(output(0), output(1), output(2), output(3));
}

// attitude helper for finding the shortest quaternion rotation from vector 1 to vector 2
Eigen::Quaterniond QH::Quat_twovec(Eigen::Vector3d* v1, Eigen::Vector3d* v2) {
	Eigen::Quaterniond output;
	Eigen::Vector3d cr = (*v1).cross(*v2);
	double dt = (*v1).dot(*v2);
	double eps = 1.0E-5;

	// handle corner cases with 180 degree rotations
	// if the two vectors are parallel, cross product is zero
	// if they point opposite, the dot product is negative
	if ((cr.norm() < eps) && (dt < 0)) {
		cr = (*v1).cwiseAbs();
		if (cr[0] < cr[1]) {
			if (cr[0] < cr[2]) {
				cr = {1, 0, 0};
			} else {
				cr = {0, 0, 1};
			}
		} else {
			if (cr[1] < cr[2]) {
				cr = {0, 1, 0};
			} else {
				cr = {0, 0, 1};
			}
		}
		output.w() = 0.0;
		cr = (*v1).cross(cr);
	// normal case, do half-way quaternion solution
	} else {
		output.w() = dt + std::sqrt(((*v1).squaredNorm() * (*v2).squaredNorm()));
	}

	output.x() = cr[0];
	output.y() = cr[1];
	output.z() = cr[2];
	output.normalize();
	return output;
}

// constrains the x and y velocities to the given limits
Eigen::Vector2d QH::ConstrainXY(Eigen::Vector2d sp_,  Eigen::Vector2d sp, double lim) {
	Eigen::Vector2d sp_add = sp_ + sp;
	Eigen::Vector2d sp_sub = sp - sp_;

	// even once added the two vectors are below the  limit
	if (sp_add.norm() <= lim) {
		return sp_add;
	}
	// the desired setpoint is larger than the limit by its self
	else if (sp_.norm() >= lim) {
		return sp_.normalized() * lim;
	}
	// the desired setpoint and the ff setpoint are equal
	else if (std::abs(sp_sub.norm()) < 0.001f) {
		return sp_.normalized() * lim;
	}
	// the desired setpointis approx 0
	else if (sp_.norm() < 0.001f) {
		return sp.normalized() * lim;
	}
	// other wise its more complicated and you need to project the ff sp onto the desired sp before adding them
	else {
		Eigen::Vector2d u = sp.normalized();
		double m = u.dot(sp_);
		double c = sp_.dot(sp_) - lim * lim;
		double s = -m + std::sqrt(m * m - c);
		return sp_ + u * s;
	}
}

// helper to return the sign of a value
int QH::signfinder(double value) {
	return ((0.0 < value) - (value < 0.0));
}

// helper converts to Euler angles in 3-2-1 sequence
void QH::ToEulerAngles(Eigen::Quaterniond *q, Eigen::Vector3d *euler) {
    // roll (x-axis rotation)
    double sinr_cosp = 2 * ((*q).w() * (*q).x() + (*q).y() * (*q).z());
    double cosr_cosp = 1 - 2 * ((*q).x() * (*q).x() + (*q).y() * (*q).y());
    (*euler)[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * ((*q).w() * (*q).y() - (*q).x() * (*q).z()));
    double cosp = std::sqrt(1 - 2 * ((*q).w() * (*q).y() - (*q).x() * (*q).z()));
    (*euler)[1] = 2 * std::atan2(sinp, cosp) - (3.14159 / 2);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * ((*q).w() * (*q).z() + (*q).x() * (*q).y());
    double cosy_cosp = 1 - 2 * ((*q).y() * (*q).y() + (*q).z() * (*q).z());
    (*euler)[2] = std::atan2(siny_cosp, cosy_cosp);
}

// calculates the obstalces current jacobian matrix using standard equations
void QH::JacobianMatrix(Eigen::Vector3d* global_apos, Eigen::Matrix3d* jac_mat) {
	double r = (*global_apos)[0];
	double p = (*global_apos)[1];
	(*jac_mat)(0, 0) = 1;
	(*jac_mat)(0, 1) = sin(r) * tan(p);
	(*jac_mat)(0, 2) = cos(r) * tan(p);
	(*jac_mat)(1, 0) = 0;
	(*jac_mat)(1, 1) = cos(r);
	(*jac_mat)(1, 2) = -sin(r);
	(*jac_mat)(2, 0) = 0;
	(*jac_mat)(2, 1) = sin(r) / cos(p);
	(*jac_mat)(2, 2) = cos(r) / cos(p);
}

// helper function for RK4_vec_update
void QH::RK4_update(double* xn, double* xn_dot, double* h) {
    // k1
    double k1 = (*xn_dot);

    // k2
    double xn_h_2 = (*xn) + ((*h) / 2) * k1;
    double k2 = (xn_h_2 - (*xn)) / ((*h) / 2);;

    // k3
    double xn_h_3 = (*xn) + ((*h) / 2) * k2;
    double k3 = (xn_h_3 - (*xn)) / ((*h) / 2);

    // k4
    double xn_h_4 = (*xn) + (*h) * k3;
    double k4 = (xn_h_4 - (*xn)) / (*h);

    // xn+1
    (*xn) = (*xn) + ((*h) / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
}

// function that entire system can use to do Runge Kutta 4 integration updates on 3 vecs
void QH::RK4_vec_update(Eigen::Vector3d* xn, Eigen::Vector3d xn_dot, double h) {
	int len = (*xn).rows();
	double cur_out = 0.0;
	double cur_xn_dot = 0.0;

	for (int i = 0; i < len; i++) {
		cur_out = (*xn)(i);
		cur_xn_dot = xn_dot(i);
    	RK4_update(&cur_out, &cur_xn_dot, &h);
		(*xn)(i) = cur_out;
	}
}

// Newton Raphton method to find the induced velocity
void QH::inducedVelocity(double* vi , double* vh, double* vz, double* vxy) {
	// first find derivative of initial guess
	double cc = 0;
	double vi_d = vi_poly_d(vi, vz, vxy);
	double lim = 1.0E-5;
	// if that derivative is too small then cancel the loop as likely already there
	if (std::abs(vi_d) < (1.0E-10)) {
		*vi = *vi;
	}
	// otherwise run the loop
	else {
		// find the initial change
		double vi_p = vi_poly(vi, vh, vz, vxy);
		double vi_change = vi_p/vi_d;
		// while the change is great than the lim keep going
		while ((std::abs(vi_change) > lim)) {
			// update the guess
			*vi = *vi - vi_change;
			// find the new change
			vi_p = vi_poly(vi, vh, vz, vxy);
			vi_d = vi_poly_d(vi, vz, vxy);
			vi_change = vi_p/vi_d;
		}
	}
}

// induced velocity polynomial function
double QH::vi_poly(double* vi, double* vh, double* vz, double* vxy) {
	double output = (std::pow((*vi), 4)) - (2*(*vz)*(std::pow((*vi), 3))) + ((std::pow((*vi), 2))*((std::pow((*vz), 2))+(std::pow((*vxy), 2)))) - (std::pow((*vh), 4));
	return output;
}

// induced velocity polynomial function derivative
double QH::vi_poly_d(double* vi, double* vz, double* vxy) {
	double output = (4*std::pow((*vi), 3)) - (6*(*vz)*(std::pow((*vi), 2))) + ((2*(*vi))*((std::pow((*vz), 2))+(std::pow((*vxy), 2))));
	return output;
}

// Newton Raphton method to find the induced velocity for required thrusts
void QH::vhinducedVelocity(double* vi , double* vi_pre, Eigen::Vector3d* vinf, double thrust, double rho, double Ar) {
	// first find derivative of initial guess
	double vi_p = vi_poly_2(vi, vinf, thrust, rho, Ar);
	double vi_check = vi_p - vi_poly_2(vi_pre, vinf, thrust, rho, Ar);
	double lim = 1.0E-5;
	// if that derivative is too small then cancel the loop as likely already there
	if (std::abs(vi_check) < (1.0E-10)) {
		*vi = *vi;
	}
	// otherwise run the loop
	else {
		// find the initial change
		vi_p = vi_poly_2(vi, vinf, thrust, rho, Ar);
		vi_check = vi_p - vi_poly_2(vi_pre, vinf, thrust, rho, Ar);
		double vi_change = vi_p*(((*vi)-(*vi_pre))/vi_check);
		// while the change is great than the lim keep going
		while ((std::abs(vi_change) > lim)) {
			// update the guess
			*vi_pre = *vi;
			*vi = *vi - vi_change;
			// find the new change
			vi_p = vi_poly_2(vi, vinf, thrust, rho, Ar);
			vi_check = vi_p - vi_poly_2(vi_pre, vinf, thrust, rho, Ar);
			vi_change = vi_p*(((*vi)-(*vi_pre))/vi_check);
		}
	}
}

// induced velocity polynomial function for required thrusts
double QH::vi_poly_2(double* vi, Eigen::Vector3d* vinf, double thrust, double rho, double Ar) {
		Eigen::Vector3d e3 = {0,0,1};
		Eigen::Vector3d vU = ((*vi)*e3) + (*vinf);
		double U = vU.norm();
		double output = (*vi) - (thrust)/(2*rho*Ar*U);
		return output;
}

// hover velocity polynomial function for required thrusts
void QH::vh_poly(double* vh, double* vi, double* vz, double* vxy) {
	double temp = (std::pow((*vxy), 2)) + (((*vi)-(*vz))*((*vi)-(*vz)));
	(*vh) = sqrt((*vi)*sqrt(temp));
}

// thrust helper
// function to help calculate how non hover flight change the thrust produced
void QH::adaptedThrusts(Eigen::Vector4d* thrusts, double rho, double Ct, double prop_dia, Eigen::Vector4d* rotors_speed, Eigen::Vector3d* global_vel, Eigen::Vector3d* body_avel, Eigen::Matrix <double, 4, 3>* r_arms, Eigen::Matrix3d* rot_mat, Eigen::Vector4d* v_i, Eigen::Vector4d* v_i_old) {
	// find the rotor area
	double Ar = 0.25 * 3.14159 * (prop_dia*prop_dia);
	double vh, vz, vxy, U;
	Eigen::Vector3d vinf;
	Eigen::Vector3d vvz;
	Eigen::Vector3d vvxy;
	Eigen::Vector3d vU;
	Eigen::Vector3d e3 = {0,0,1};
	// for each rotor
	for (int i = 0; i < 4; i++) {
		// find the basic hover thrust and related hover velocity
		(*thrusts)[i] = rho * Ct * (std::pow(prop_dia, 4)) * ((*rotors_speed)[i]*(*rotors_speed)[i]);
		vh = sqrt(((*thrusts)[i])/(2*rho*Ar));
		//((*v_i)[i]) = vh;
		// use drone state to find freestream velocity and components
		vinf = Eigen::Matrix3d::Identity()*((((*rot_mat).transpose())*(*global_vel)) + ((*body_avel).cross((*r_arms).row(i))));
		vvz = {0,0,vinf[2]};
		vvxy = vinf - vvz;
		vz = vvz.norm();
		vxy = vvxy.norm();
		// find vi
		inducedVelocity(&((*v_i)[i]), &vh, &vz, &vxy);
		// find the adjusted thrust
		vU = ((*v_i)[i]*e3) + vinf;
		U = vU.norm();
		(*thrusts)[i] = 2 * rho * Ar * (*v_i)[i] * U;
	}
}

// function to help calculate how non hover flight changes the thrust required
void QH::adaptedreqThrusts(Eigen::Vector4d* thrusts, double rho, double prop_dia, Eigen::Vector3d* global_vel, Eigen::Vector3d* body_avel, Eigen::Matrix <double, 4, 3>* r_arms, Eigen::Matrix3d* rot_mat, Eigen::Vector4d* v_i_s, Eigen::Vector4d* v_i_s_old) {
	// find the rotor area
	double Ar = 0.25 * 3.14159 * (prop_dia*prop_dia);
	double vh, vz, vxy, thrust_adjust;
	Eigen::Vector3d vinf;
	Eigen::Vector3d vvz;
	Eigen::Vector3d vvxy;
	Eigen::Vector3d vU;
	Eigen::Vector3d e3 = {0,0,1};
	// for each rotor
	for (int i = 0; i < 4; i++) {
		// grab the inputted og thrust
		thrust_adjust = std::abs((*thrusts)[i]);
		// use drone state to find freestream velocity and components
		vinf = Eigen::Matrix3d::Identity()*((((*rot_mat).transpose())*(*global_vel)) + ((*body_avel).cross((*r_arms).row(i))));
		vvz = {0,0,vinf[2]};
		vvxy = vinf - vvz;
		vz = vvz.norm();
		vxy = vvxy.norm();
		// find vi
		vhinducedVelocity(&((*v_i_s)[i]), &((*v_i_s_old)[i]), &vinf, thrust_adjust, rho, Ar);

		// use vi to find vh
		vh_poly(&vh, &((*v_i_s)[i]), &vz, &vxy);
		// find the adjusted thrust
		(*thrusts)[i] = copysignf(((vh*vh)*2*rho*Ar), (*thrusts)[i]);
	}
}