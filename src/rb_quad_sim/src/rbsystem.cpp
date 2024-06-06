#include "RBsystem.h"

typedef std::chrono::high_resolution_clock clocky;

// constructor for initializing a rigid body system given a drone and obstacle as well as 6 planes
void RBsystem::RBsystem::initrb(float freqsim, Quadcopter::dr_prop* drone_props, RBH::planes* planes, double h, double stiff, double damp) {
	// initialize the drone and planes
	drone = Quadcopter::UAS(drone_props, h);
	plane_eqns = *planes;

	// system sparse matrices
	//const int size = 48;
	const int size = 42; // 6 for drone, then 6 for 6 walls, 7 * 6 = 42
	// in star frame of reference 7 for drone, then 7 for 6 walls, 7 * 7 = 49
	An_mat = Eigen::SparseMatrix <double>(6,size);
	An_mat_dot = Eigen::SparseMatrix <double>(6, size);
	M_mat = Eigen::SparseMatrix <double>(size, size);
	M_mat_pre = Eigen::SparseMatrix <double>(size, size);
	M_mat_dot = Eigen::SparseMatrix <double>(size, size);
	M_mat_inv = Eigen::SparseMatrix <double>(size, size);
	M_mat_star = Eigen::SparseMatrix <double>(49, 49);
	O_mat = Eigen::SparseMatrix <double>(size, size);
	O_mat_star = Eigen::SparseMatrix <double>(49, 49);
	N_mat = Eigen::SparseMatrix <double>(49, size);
	N_mat_dot = Eigen::SparseMatrix <double>(49, size);
	imposed_vel = Eigen::SparseMatrix <double>(1, size);
	global_vel = Eigen::SparseMatrix <double>(1, size);
	q_dot = Eigen::SparseMatrix <double>(1, 49);

	// Matrices used in calculations
	Lambda_i.setZero();
	drphins = { 0,0,0,0,0,0 };
	drdphins = { 0,0,0,0,0,0 };
	dr_vel_lims = { drone_props->PIDlims[6], drone_props->PIDlims[7], drone_props->PIDlims[8] };

	// sim properties
	step_size = h;
	sigd = 6;
	drplnum = 0;

	// physical properties
	stiffness = stiff;
	damping = damp;
	rho = 1.225;

	// interaction properties
	drphin = 0;
	drdphin = 0;

	// interaction flags
	contact = false;
	pre_contact = false;
	post_contact = false;
	no_contact = true;

	mats_r = false;

	// numerical variables
	count = 0;
	i = 0;
	freq = 0.0;
	pos_norm = 0;

	DDP = { 0.0,0.0,1.0 };
	Wvv = { 0,0,0 };
	Fid = { 0,0,0 };
	Int_for = { 0,0,0 };
	droneCrvel = { 0,0,0,0,0,0 };
	pos_diff = { 0,0,0 };

	// fills the sparse matrices with zeros where there will be values
	fill_matrices();
	// fills the mass matrix as it doesn't change during the simulation
	RBH::massMatrix(&M_mat, &M_mat_star, &M_mat_inv, &M_mat_dot, &M_mat_pre, &N_mat, &drone, step_size);

	//set up mass matrices
	plane_interaction(&imposed_vel, &Int_for, &drphins, &drdphins, &Lambda_i, &An_mat, &An_mat_dot, &M_mat, &M_mat_inv, &M_mat_dot, &M_mat_star, &O_mat, &O_mat_star, &N_mat, &N_mat_dot, &global_vel, &plane_eqns, &drone, stiffness, damping, step_size, &contact, &pre_contact, &post_contact, &no_contact);
}

// fills all the objects sparse matrices with zeros where there will eventually be values
// uses the helper function sparse_fill
void RBsystem::RBsystem::fill_matrices() {
	Eigen::Matrix3d place_holder = Eigen::Matrix3d::Zero();
	Eigen::Matrix4d place_holder4 = Eigen::Matrix4d::Zero();
	const int size = 42;
	// fill An_mat
	for (int i=0; i < size; i++) {
		An_mat.insert(0, i) = 0;
		An_mat.insert(1, i) = 0;
		An_mat.insert(2, i) = 0;
		An_mat.insert(3, i) = 0;
		An_mat.insert(4, i) = 0;
		An_mat.insert(5, i) = 0;
	}
	// fill An_mat_dot
	for (int i = 0; i < size; i++) {
		An_mat_dot.insert(0, i) = 0;
		An_mat_dot.insert(1, i) = 0;
		An_mat_dot.insert(2, i) = 0;
		An_mat_dot.insert(3, i) = 0;
		An_mat_dot.insert(4, i) = 0;
		An_mat_dot.insert(5, i) = 0;
	}
	//fill imposed_vel
	for (int i = 0; i < size; i++) {
		imposed_vel.insert(0, i) = 0;
	}
	//fill global vel
	for (int i = 0; i < size; i++) {
		global_vel.insert(0, i) = 0;
	}
	//fill q_dot
	for (int i = 0; i < 49; i++) {
		q_dot.insert(0, i) = 0;
	}
	// fill M_mat
	for (int i = 0; i < size/6; i++) {
		RBH::sparse_fill(&M_mat, &place_holder, 3 * i, 3 * i);
	}
	// fill M_mat_pre
	for (int i = 0; i < size/6; i++) {
		RBH::sparse_fill(&M_mat_pre, &place_holder, 3 * i, 3 * i);
	}
	//fill M_mat_dot
	for (int i = 0; i < size/6; i++) {
		RBH::sparse_fill(&M_mat_dot, &place_holder, 3 * i, 3 * i);
	}
	// fill M_mat_inv
	for (int i = 0; i < size/6; i++) {
		RBH::sparse_fill(&M_mat_inv, &place_holder, 3 * i, 3 * i);
	}
	// fill M_mat_star
	for (int i = 0; i < 7; i++) {
		RBH::sparse_fill(&M_mat_star, &place_holder, 7 * i, 7 * i);
		RBH::sparse_fill4(&M_mat_star, &place_holder4, (7 * i + 3), (7 * i + 3));
	}
	// fill O_mat
	for (int i = 0; i < size/6; i++) {
		RBH::sparse_fill(&O_mat, &place_holder, 3 * i, 3 * i);
	}
	// fill O_mat_star
	for (int i = 0; i < 7; i++) {
		RBH::sparse_fill(&O_mat_star, &place_holder, 7 * i, 7 * i);
		RBH::sparse_fill4(&O_mat_star, &place_holder4, (7 * i + 3), (7 * i + 3));
	}
	// fill N_mat !!!
	for (int i = 0; i < 7; i++) {
		RBH::sparse_fill(&N_mat, &place_holder, 7 * i, 6 * i);
	}
	// fill N_mat_dot !!!
	for (int i = 0; i < 7; i++) {
		RBH::sparse_fill(&N_mat_dot, &place_holder, 7 * i, 6 * i);
	}
}

// update the state of the system
void RBsystem::RBsystem::RBstep() {
	// system dynamics update for plane contact
	plane_interaction(&imposed_vel, &Int_for, &drphins, &drdphins, &Lambda_i, &An_mat, &An_mat_dot, &M_mat, &M_mat_inv, &M_mat_dot, &M_mat_star, &O_mat, &O_mat_star, &N_mat, &N_mat_dot, &global_vel, &plane_eqns, &drone, stiffness, damping, step_size, &contact, &pre_contact, &post_contact, &no_contact);
	// std::cout << "DDP Raw: " << DDP[0] << ", " << DDP[1] << ", " << DDP[2] << std::endl;
	// std::cout << "Phins: " << drphins.transpose() << std::endl;
	// std::cout << "DPhins: " << drdphins.transpose() << std::endl;
	// std::cout << "Pre Contact: " << pre_contact << " Contact: " << contact << " Post Contact: " << post_contact << " No Contact: " << no_contact << std::endl;
	// if there was plane contact than apply the calculated velocity to the drone
	if ((abs(imposed_vel.sum())) > 0.01) {
		velimp = Eigen::MatrixXd(imposed_vel);
		droneCrvel[0] = velimp(0);
		droneCrvel[1] = velimp(1);
		droneCrvel[2] = velimp(2);
		droneCrvel[0] = std::min(droneCrvel[0], dr_vel_lims[0]);
		droneCrvel[0] = std::max(droneCrvel[0], -1 * dr_vel_lims[0]);
		droneCrvel[1] = std::min(droneCrvel[1], dr_vel_lims[0]);
		droneCrvel[1] = std::max(droneCrvel[1], -1 * dr_vel_lims[0]);
		droneCrvel[2] = std::min(droneCrvel[2], dr_vel_lims[1]);
		droneCrvel[2] = std::max(droneCrvel[2], -1 * dr_vel_lims[2]);
		droneCrvel = droneCrvel * -1;
		drone.CR_update(&droneCrvel);
		//reset imposed vel
		// for (int i = 0; i < 42; i++) {
		// 	imposed_vel.coeffRef(0, i) = 0;
		// }
		for (int i=0; i<imposed_vel.outerSize(); ++i)
			for (Eigen::SparseMatrix<double>::InnerIterator it(imposed_vel,i); it; ++it) {
				it.valueRef() = 0;
			}
		velimp = Eigen::MatrixXd(imposed_vel);
		droneCrvel[0] = velimp(0);
		droneCrvel[1] = velimp(1);
		droneCrvel[2] = velimp(2);
		// tell the drone to stay still until ddp is close again
		drone.set_DDP();
	} else {
		//drone control update
		drone.Controller(&DDP, &contact, &pre_contact, &post_contact, &no_contact);
		drone.Mixer();
		// drone dynamics update
		Fid = drone.wind_force(&Wvv);
		dgout = drone.Accelerator(&Fid);
		// drone state update
		drone.update_state();
		Lambda_i = dgout.F_internal;
	}

	// outputs the drones body frame velocities
	global_vel.coeffRef(0, 0) = drone.get_state().g_vel[0];
	global_vel.coeffRef(0, 1) = drone.get_state().g_vel[1];
	global_vel.coeffRef(0, 2) = drone.get_state().g_vel[2];
	global_vel.coeffRef(0, 3) = drone.get_state().g_avel[0];
	global_vel.coeffRef(0, 4) = drone.get_state().g_avel[1];
	global_vel.coeffRef(0, 5) = drone.get_state().g_avel[2];

	// pushing outputs from the loop
	commsmsgs::msg::Rbquadsimpub msg{};
	msg.header.stamp = this->now();
	msg.position = {drone.get_state().g_pos[0], drone.get_state().g_pos[1], drone.get_state().g_pos[2]};
	msg.velocity = {drone.get_state().g_vel[0], drone.get_state().g_vel[1], drone.get_state().g_vel[2]};
	msg.acceleration = {drone.get_state().g_acc[0], drone.get_state().g_acc[1], drone.get_state().g_acc[2]};
	msg.orientation = {drone.get_state().g_att.w(), drone.get_state().g_att.x(), drone.get_state().g_att.y(), drone.get_state().g_att.z()};
	msg.angular_velocity = {drone.get_state().b_avel[0], drone.get_state().b_avel[1], drone.get_state().b_avel[2]};
	// pose / orientation ???
	msg.drag = {dgout.Fd[0], dgout.Fd[1], dgout.Fd[2]};
	msg.gravity = {dgout.Fg[0], dgout.Fg[1], dgout.Fg[2]};
	msg.interaction = {Int_for[0], Int_for[1], Int_for[2]};
	msg.rotation_matrix.m11 = drone.get_state().r_mat(0, 0);
	msg.rotation_matrix.m12 = drone.get_state().r_mat(0, 1);
	msg.rotation_matrix.m13 = drone.get_state().r_mat(0, 2);
	msg.rotation_matrix.m21 = drone.get_state().r_mat(1, 0);
	msg.rotation_matrix.m22 = drone.get_state().r_mat(1, 1);
	msg.rotation_matrix.m23 = drone.get_state().r_mat(1, 2);
	msg.rotation_matrix.m31 = drone.get_state().r_mat(2, 0);
	msg.rotation_matrix.m32 = drone.get_state().r_mat(2, 1);
	msg.rotation_matrix.m33 = drone.get_state().r_mat(2, 2);

	RBH::matrix_to_msg(&M_mat_inv, &msg.m_inv);
	RBH::matrix_to_msg(&An_mat, &msg.ac);
	RBH::matrix_to_msg(&global_vel, &msg.vg);

	msg.contact = contact;
	msg.pre_contact = pre_contact;
	msg.post_contact = post_contact;
	msg.no_contact = no_contact;
	msg.sim_freq = freq;

	// publish the message
	rbquadsim_publisher_->publish(msg);

	std::chrono::duration<double> dt = clocky::now() - start_time;
	freq = 1 / dt.count();
	while (freq > frim)
	{
		dt = clocky::now() - start_time;
		freq = 1 / dt.count();
	}
	start_time = clocky::now();

	// if (count % 10 == 0) {
	// 	//std::cout << "M mat ready?: " << M_mat_inv.nonZeros() << std::endl;
	// 	//std::cout << "SIM Frequency: " << freq << std::endl;
	// 	std::cout << "Desired Drone Position: " << DDP[0] << ", " << DDP[1] << ", " << DDP[2] << std::endl;
	// 	//std::cout << "Drone Position SIM: " << drone.get_state().g_pos[0] << ", " << drone.get_state().g_pos[1] << ", " << drone.get_state().g_pos[2] << std::endl;
	// }
	count += 1;
	i += 1;

	// check if the matrices are ready
	if (M_mat_inv.nonZeros() > 0) {
		mats_r = true;
	}
}

// function to calculate an interaction between the drone an a plane
void RBsystem::RBsystem::plane_interaction(Eigen::SparseMatrix<double>* vel_imposed, Eigen::Vector3d* Int_for, Eigen::Matrix<double, 6, 1>* phins, Eigen::Matrix<double, 6, 1>* dphins, Eigen::Vector3d* Li, Eigen::SparseMatrix<double>* mat_An, Eigen::SparseMatrix<double>* mat_dot_An, Eigen::SparseMatrix<double>* mat_M, Eigen::SparseMatrix<double>* mat_inv_M, Eigen::SparseMatrix<double>* mat_dot_M, Eigen::SparseMatrix<double>* mat_star_M, Eigen::SparseMatrix<double>* mat_O, Eigen::SparseMatrix<double>* mat_star_O, Eigen::SparseMatrix<double>* mat_N, Eigen::SparseMatrix<double>* mat_dot_N, Eigen::SparseMatrix<double>* vel_globe, RBH::planes* eqns_plane, Quadcopter::UAS* Udrone, double stiffness, double damping, double s_s, bool* contact, bool* pre_contact, bool* post_contact, bool* no_contact) {
	// setting up some temp variables
	bool ConRel = true;
	Eigen::Matrix <double, 6, 1> lambdan;
	Eigen::Matrix <double, 6, 1> lambdai;
	Eigen::Matrix <double, 42, 1> Fint;
	Eigen::Vector3d dpos = (*Udrone).get_state().g_pos;

	// setup all the plane interface variables
    RBH::plane_interface_jacobian(mat_An, eqns_plane);
	RBH::plane_interface_distance(&dpos, phins, eqns_plane, (*Udrone).get_state().con_rad);
    RBH::plane_interface_velocity(mat_An, vel_globe, dphins);

	// setup all the sparse matrices
	RBH::transformMatrix(mat_N, Udrone);
    RBH::dotTransform(mat_dot_N, Udrone);
	RBH::massMatrix(&M_mat, &M_mat_star, &M_mat_inv, &M_mat_dot, &M_mat_pre, &N_mat, &drone, step_size);
	RBH::omegaMatrix(mat_O, mat_star_O, mat_N, mat_dot_N, mat_star_M, Udrone);
    RBH::dotJacobian(mat_dot_An, eqns_plane);

	// setting up some local variables to modify
	Eigen::Matrix <double, 6, 1> drphins_l = (*phins);
	Eigen::Matrix <double, 6, 1> drdphins_l = (*dphins);

	// import the force from the plane
	lambdai[0] = (*Li)[0];
	lambdai[1] = (*Li)[0];
	lambdai[2] = (*Li)[1];
	lambdai[3] = (*Li)[1];
	lambdai[4] = (*Li)[2];
	lambdai[5] = (*Li)[2];

	int phini = 0;
	double minphin = 0.0;

	// contact flaging for pre-contact
	minphin = (*phins).minCoeff(&phini);
	if ((minphin > 0) && (minphin <= std::abs(1 * s_s * (*dphins)[phini])) && ((*dphins)[phini] < 0) && ((*no_contact) == true)){
		(*pre_contact) = true;
		(*no_contact) = false;
	}

	// if the drone is not in contact with the plane
	//std::cout << "sim phins: " << (*phins).transpose() << std::endl;
	if (((*phins).minCoeff() > 0) || ((*phins).maxCoeff() < -0.9)) {
		lambdan = { 0,0,0,0,0,0 };
		//(*contact) = false;
	}
	// if the drone is in contact with a plane
	else if ((*phins).minCoeff() < 0) {
		for (int i = 0; i < 6; i++) {
			drphins_l[i] = std::min(0.0, drphins_l[i]);
			if (drphins_l[i] == 0.0) { drdphins_l[i] = 0.0; }
		}
		// if constitutive relations are being used
		if (ConRel) {
			// calculate the force from the plane
			// !!! spring and damper force may not always act in same direction !!!
			lambdan = -1 * stiffness * drphins_l - damping * drdphins_l;
			for (int i = 0; i < 6; i++) {
				if (lambdan[i] == 0.0) { lambdai[i] = 0.0; }
			}
			//std::cout << "lambdan: " << lambdan[0] << ", " << lambdan[1] << ", " << lambdan[2] << ", " << lambdan[3] << ", " << lambdan[4] << ", " << lambdan[5] << std::endl;
			//std::cout << "lambdai: " << lambdai[0] << ", " << lambdai[1] << ", " << lambdai[2] << ", " << lambdai[3] << ", " << lambdai[4] << ", " << lambdai[5] << std::endl;
			// calculate the total force on the drone
			lambdai += lambdan;
			// calculate the velocity to be imposed on the drone
			interface_solution_CR(&lambdai, mat_M, mat_inv_M, mat_dot_M, mat_An, mat_dot_An, mat_O, dphins, phins, s_s, vel_imposed);
		}
		if ((*pre_contact) == true) {
			(*contact) = true;
			(*pre_contact) = false;
		}
	}

	// contact flaging for post-contact
	minphin = (*phins).minCoeff(&phini);
	if ((minphin >= std::abs(1 * s_s * (*dphins)[phini])) && ((*dphins)[phini] >= 0) && ((*contact) == true)) {
		(*post_contact) = true;
		(*contact) = false;
	}

	// contact flagging for no contact
	if ((minphin >= std::abs(2 * s_s * (*dphins)[phini])) && ((*dphins)[phini] >= 0) && ((*post_contact) == true)) {
		(*no_contact) = true;
		(*post_contact) = false;
	}

	// output the calculated info
	Fint = (*mat_An).transpose() * lambdan;
	(*Int_for)[0] = Fint[0];
	(*Int_for)[1] = Fint[1];
	(*Int_for)[2] = Fint[2];
	(*Int_for) = (*Udrone).get_state().r_mat * (*Int_for);
}

// calculates the result of a constitutive relations based rigid body reaction using the impulse momentum level equations
void RBsystem::RBsystem::interface_solution_CR(Eigen::Matrix <double, 6, 1>* nlambda, Eigen::SparseMatrix<double>* mat_M, Eigen::SparseMatrix<double>* mat_inv_M, Eigen::SparseMatrix<double>* mat_dot_M, Eigen::SparseMatrix<double>* mat_An, Eigen::SparseMatrix<double>* mat_dot_An, Eigen::SparseMatrix<double>* mat_O, Eigen::Matrix <double, 6, 1>* dphins, Eigen::Matrix <double, 6, 1>* phins, double s_s, Eigen::SparseMatrix<double>* V) {
	const int size = 42;
	// sset up some helper variables
	Eigen::SparseMatrix <double> Mhat(size, size);
	Eigen::SparseMatrix <double> Mhat_inv(size, size);
	Eigen::SparseMatrix <double> Phat(size, size);
	Eigen::SparseMatrix <double> eye(size, size);
	Eigen::SparseMatrix <double> temp(size, size);
	Eigen::SparseMatrix <double> bhat(6, size);

	// calculates the Mhat matrix
	Mhat = (*mat_M) - (s_s * s_s / 2) * (*mat_O) * (*mat_inv_M) * (*mat_O);
	for (int i = 0; i < size; i++) {
		eye.insert(i, i) = 1;
	}
	// calculates the Mhat_inv inverse
	Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solvermhat;
	solvermhat.compute(Mhat);
	temp = solvermhat.solve(eye);
	Mhat_inv = solvermhat.solve(temp);

	// calculates the Phat vector
	Phat = (*mat_M) - s_s * (*mat_O) * (eye - (s_s / 2) * (*mat_inv_M) * (*mat_dot_M));
	
	// calculates the bhat vector
	bhat = (*mat_An) * Mhat_inv * Phat + s_s * (*mat_dot_An);

	// calcualates the rhs of the equation
	Eigen::Matrix <double, 6, 6> mid;
	//std::cout << "Mhat_inv: " << Mhat_inv << std::endl;
	//std::cout << "mat_An: " << (*mat_An) << std::endl;
	mid = ((*mat_An) * Mhat_inv * (*mat_An).transpose());
	//std::cout << "mid: " << mid << std::endl;
	//std::cout << "nlambda: " << (*nlambda).transpose() << std::endl;
	Eigen::Matrix <double, 6, 1> bbb = (s_s * mid * (*nlambda));
	//std::cout << "dphins: " << (*dphins).transpose() << std::endl;
	//std::cout << "bbb: " << bbb.transpose() << std::endl;
	Eigen::VectorXd rhs = (*dphins) - bbb;

	//std::cout << "bhat: " << bhat.block(0,0,6,size) << std::endl;
	//std::cout << "rhs: " << rhs.transpose() << std::endl;

	// replace any near zero or nan bhat vals with zero
	//
	//

	// finally solves for the velocity to be imposed on the drone from each plane
	// currently using a for loop but should be calculated properly using a pivoting method
	// bhat * V = rhss
	// 6x42 * 42x1 = 6x1
	Eigen::VectorXd Vlocal;
	(*V).setZero();
	// for (int i = 0; i < 6; i++) {
	// 	if ((*phins)[i] < 0) {
	// 		(*V) = (*V) + bhat.block(i, 0, 1, size) / rhs[i];
	// 	}
	// }

	Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solversparse;
	solversparse.compute(bhat);
	if (solversparse.info() != Eigen::Success) {
		std::cout << "decomp failed" << std::endl;
	}
	Vlocal = solversparse.solve(rhs);
	if (solversparse.info() != Eigen::Success) {
		std::cout << "solve failed" << std::endl;
	}
	(*V) = Vlocal.sparseView();
	//std::cout << "success" << std::endl;

	//std::cout << "V: " << Vlocal << std::endl;
}

// brim callback
void RBsystem::RBsystem::brim_callback(const commsmsgs::msg::Brimpub::UniquePtr & msg) {
	DDP[0] = msg->desired_drone_position[0];
	DDP[1] = msg->desired_drone_position[1];
	DDP[2] = msg->desired_drone_position[2];
}

// prim callback
void RBsystem::RBsystem::prim_callback(const commsmsgs::msg::Rrimpub::UniquePtr & msg) {
	//do nothing for now
}