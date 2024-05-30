#include <../include/b_rim/brim.h>

using namespace std::chrono_literals;
typedef std::chrono::high_resolution_clock clocky;

// BRIM constructor
void BRIM::brim::initbrim(float fr, int fcom1, int fcom2, int rim_type, float xlim, float ylim, float zlim, float dxlim, float dylim, float dzlim) {
	const int size = 42;
	const int dim = 3;
	// initialize the sparse matrices
	Ac = Eigen::SparseMatrix <double>(6, size);
	M_hat_inv = Eigen::SparseMatrix <double>(size, size);
	Pc_hat = Eigen::SparseMatrix <double>(size, size);
	Ai = Eigen::SparseMatrix <double>(dim, size);
	Ai_old = Eigen::SparseMatrix <double>(dim, size);
	Ai_dot = Eigen::SparseMatrix <double>(dim, size);
	I = Eigen::SparseMatrix <double>(size, size);
	IPMi = Eigen::SparseMatrix <double>(size, size);

	// set all dense matrices to zero
	M_tilde = Eigen::Matrix <double, dim, dim>::Zero();
	M_tilde_inv = Eigen::Matrix <double, dim, dim>::Zero();
	lambda_tilde = Eigen::Matrix <double, dim, 1>::Zero();
	lambda_i = Eigen::Matrix <double, dim, 1>::Zero();
	phin_list = Eigen::Matrix<double, dim, 1>::Zero();
	dot_phin_list = Eigen::Matrix<double, dim, 1>::Zero();
	f_ext = Eigen::Matrix<double, size, 1>::Zero();
	v_g = Eigen::Matrix<double, size, 1>::Zero();
	v_g_old = Eigen::Matrix<double, size, 1>::Zero();
	R_vec = Eigen::Vector3d::Zero();
	R_vec_est = Eigen::Vector3d::Zero();
	R_tilde_mat = Eigen::Matrix3d::Zero();
	DDP = {0,0,1.0};
	DP = {0,0,1.0};
	M_temp_offset = Eigen::Matrix3d::Zero();

	fg = Eigen::Vector3d::Zero();
	fd = Eigen::Vector3d::Zero();
	fi = Eigen::Vector3d::Zero();

	frim = fr;
	h = 1. / fr;
	h_com1 = 1. / fcom1;
	r_type = rim_type;

	count = 0;
	i = 0;
	freq = 0.0;

	lims = {xlim, ylim, zlim};
	dlims = {dxlim, dylim, dzlim};

	// fill the sparse matrices
	init_mats();
}

// Main BRIM loop that updates the state of the RIM during the program run
void BRIM::brim::BRIMstep() {
	// start flag?
	// run = ptr->running;
	// mystery
	// ptr->DDC = temp_DDP;
	// params

	auto loop_start = clocky::now();

	// setup all the RB stuff and get first time estimate run first 100 loops to ensure all the data was imported properly
	if (i < 100) {
		rb_setup(&Ac, &M_hat_inv, &Pc_hat, &I, &IPMi, &M_temp_offset);
	}

	// runs the main update function every loop which updates the BMN system vectors
	bmn_update(&r_type, &phin_list, &dot_phin_list, &lambda_tilde, &lambda_i, &R_vec_est, &M_tilde_inv, h, &dlims, &lims);

	// publishs the BRIM data
	commsmsgs::msg::Brimpub msg{};
	msg.header.stamp = this->now();
	msg.phin_list = {phin_list[0], phin_list[1], phin_list[2]};
	msg.dot_phin_list = {dot_phin_list[0], dot_phin_list[1], dot_phin_list[2]};
	msg.desired_drone_position = {DDP[0], DDP[1], DDP[2]};
	msg.brim_freq = freq;
	msg.brim_count = count;
	msg.brim_time = loop_time.count();
	brim_publisher_->publish(msg);

	// calculates loop time
	auto loop_end = clocky::now();
	loop_time = loop_end - loop_start;

	// no more sim stuff below this line //
	// frequency limiter
	std::chrono::duration<double> dt = clocky::now() - start_time;
	freq = 1 / dt.count();
	while (freq > frim)
	{
		dt = clocky::now() - start_time;
		freq = 1 / dt.count();
	}
	start_time = clocky::now();

	// if (count % 100 == 0) {
	// 	std::cout << "Simulated Drone Position: " << DP[0] << ", " << DP[1] << ", " << DP[2] << std::endl;
	// 	std::cout << "Desired Drone Position: " << DDP[0] << ", " << DDP[1] << ", " << DDP[2] << std::endl;
	// 	std::cout << "phins: " << phin_list[0] << ", " << phin_list[1] << ", " << phin_list[2] << std::endl;
	// 	std::cout << "dot_phins: " << dot_phin_list[0] << ", " << dot_phin_list[1] << ", " << dot_phin_list[2] << std::endl;
	// 	std::cout << "lambda_i: " << lambda_i[0] << ", " << lambda_i[1] << ", " << lambda_i[2] << std::endl;
	// 	std::cout << "lambda_tilde: " << lambda_tilde[0] << ", " << lambda_tilde[1] << ", " << lambda_tilde[2] << std::endl;
	// 	// std::cout << "RIM Frequency: " << freq << std::endl;
	// }

	count += 1;
	i += 1;
}

// callback for the bmn subscriber
void BRIM::brim::bmn_callback(const commsmsgs::msg::bmnpub::UniquePtr & msg) {
	lambda_i = {msg->interface_force_list->x, msg->interface_force_list->y, msg->interface_force_list->z};
	DDP = {msg->desired_drone_position->x, msg->desired_drone_position->y, msg->desired_drone_position->z};
}

// callback for the rbquadsim subscriber
void BRIM::brim::rbquadsim_callback(const commsmsgs::msg::rbquadsimpub::UniquePtr & msg) {
	DP = {msg->position->x, msg->position->y, msg->position->z};
	Eigen::SparseMatrix<double> vgtemp;
	BRIM::brim::msg_to_matrix(msg->vg, &vgtemp);
	v_g = vgtemp.toDense();

	if (r_type == 2) {
		fd = {msg->drag->x, msg->drag->y, msg->drag->z};
		fg = {msg->gravity->x, msg->gravity->y, msg->gravity->z};
		fi = {msg->interaction->x, msg->interaction->y, msg->interaction->z};
		// sparse stuff
		BRIM::brim::msg_to_matrix(msg->Ac, &Ac);
		BRIM::brim::msg_to_matrix(msg->M_inv, &M_hat_inv);
		// update the rigidbody system matrices
		rb_update(&DP, &DDP, &R_vec, &R_tilde_mat, &phin_list, &dot_phin_list, &Ai, &Ai_old, &Ai_dot, &IPMi, &v_g, h, &fd, &fg, &M_temp_offset, &M_tilde, &M_tilde_inv, &f_ext, &lambda_tilde, &lambda_i, &Pc_hat, &v_g_old, h_com1, &fi);
	}
	else {
		// for ZOH and FOH
		dot_phin_list = Ai * v_g;
		R_vec = DP - DDP;
		phin_list[0] = R_vec[0];
		phin_list[1] = R_vec[1];
		phin_list[2] = R_vec[2];
	}
}

// initializes and fills all the sparsematrices with temp values
void BRIM::brim::init_mats() {
	Eigen::Matrix3d place_holder = Eigen::Matrix3d::Zero();
	const int size = 42;
	// Ac
	for (int i = 0; i < size; i++) {
		Ac.insert(0, i) = 0;
		Ac.insert(1, i) = 0;
		Ac.insert(2, i) = 0;
	}
	// M_hat_inv
	for (int i = 0; i < size / 6; i++) {
		sparse_fill(&M_hat_inv, &place_holder, 3 * i, 3 * i);
	}
	// Pc_hat
	for (int i = 0; i < size / 6; i++) {
		sparse_fill(&Pc_hat, &place_holder, 3 * i, 3 * i);
	}
	// identity matrix
	for (int i = 0; i < size; i++) {
		I.insert(i, i) = 1;
	}
	// Ai
	for (int i = 0; i < size; i++) {
		Ai.insert(0, i) = 0;
		Ai.insert(1, i) = 0;
		Ai.insert(2, i) = 0;
	}
	Ai.coeffRef(0, 0) = 1;
	Ai.coeffRef(1, 1) = 1;
	Ai.coeffRef(2, 2) = 1;
	// Ai_old
	Ai_old = Ai;
	// Ai_dot
	for (int i = 0; i < size; i++) {
		Ai_dot.insert(0, i) = 0;
		Ai_dot.insert(1, i) = 0;
		Ai_dot.insert(2, i) = 0;
	}
}

// calculates the current interface jacobian using the r vector
void BRIM::brim::Interface_Jacobian(Eigen::Vector3d* DP, Eigen::Vector3d* DDP, Eigen::Vector3d* R_vec, Eigen::Matrix3d* R_tilde_mat, Eigen::Vector3d* phin_list, Eigen::SparseMatrix<double>* Ai, Eigen::SparseMatrix<double>* Ai_old, Eigen::SparseMatrix<double>* Ai_dot, double h) {
	// first calculate the interface vector between the desired position and the actual position
	(*R_vec) = (*DP) - (*DDP);
	(*phin_list)[0] = (*R_vec)[0];
	(*phin_list)[1] = (*R_vec)[1];
	(*phin_list)[2] = (*R_vec)[2];
	// turn that vector into a skew-symmetric equivalent
	(*R_tilde_mat)(0, 1) = -(*R_vec)[2];
	(*R_tilde_mat)(0, 2) = (*R_vec)[1];
	(*R_tilde_mat)(1, 0) = (*R_vec)[2];
	(*R_tilde_mat)(1, 2) = -(*R_vec)[0];
	(*R_tilde_mat)(2, 0) = -(*R_vec)[1];
	(*R_tilde_mat)(2, 1) = (*R_vec)[0];

	// insert the SS R matrix into the interface jacobian sparse matrix
	sparse_replace(Ai, R_tilde_mat, 0, 3);
	(*Ai_dot) = ((*Ai) - (*Ai_old))/h;
	*Ai_old = *Ai;
}

// calculates Pc_hat which doesn't change during the simulation
void BRIM::brim::rb_setup(Eigen::SparseMatrix<double>* Ac, Eigen::SparseMatrix<double>* M_hat_inv, Eigen::SparseMatrix<double>* Pc_hat, Eigen::SparseMatrix<double>* I, Eigen::SparseMatrix<double>* IPMi, Eigen::Matrix3d* M_temp_offset) {
	// find projected version of the inverse mass matrix
	//std::cout << "Ac: " << (*Ac).block(0,0,6,42) << std::endl;
	//std::cout << "M_hat_inv: " << (*M_hat_inv).block(0,0,42,42) << std::endl;
	Eigen::Matrix<double, 6, 6> temp = (*Ac) * (*M_hat_inv) * (*Ac).transpose();
	Eigen::Matrix<double, 6, 6> temp_inv = temp.inverse();
	Eigen::SparseMatrix <double> ts;
	ts = Eigen::SparseMatrix <double>(6, 6);
	ts = temp_inv.sparseView();
	//std::cout << "ts: " << ts.block(0,0,6,6) << std::endl;
	// use it to calculate the projection matrix
	(*Pc_hat) = (*M_hat_inv) * (*Ac).transpose() * ts * (*Ac);
	// find the inverse of the projection matrix
	(*IPMi) = ((*I) - (*Pc_hat)) * (*M_hat_inv);
	(*M_temp_offset) = Eigen::MatrixXd::Identity(3, 3) * 1e-08;

	//std::cout << "Pc_hat: " << (*Pc_hat).block(0,0,42,42) << std::endl;
	//std::cout << "IPMi: " << (*IPMi).block(0,0,42,42) << std::endl;
}

// updates all the matrices that change when the interface jacobian changes
void BRIM::brim::rb_update(Eigen::Vector3d* DP, Eigen::Vector3d* DDP, Eigen::Vector3d* R_vec, Eigen::Matrix3d* R_tilde_mat, Eigen::Vector3d* phin_list, Eigen::Vector3d* dot_phin_list, Eigen::SparseMatrix<double>* Ai, Eigen::SparseMatrix<double>* Ai_old, Eigen::SparseMatrix<double>* Ai_dot, Eigen::SparseMatrix<double>* IPMi, Eigen::Matrix<double, 42, 1>* v_g, double h, Eigen::Vector3d* Drag, Eigen::Vector3d* Gravity, Eigen::Matrix3d* M_temp_offset, Eigen::Matrix3d* M_tilde, Eigen::Matrix3d* M_tilde_inv, Eigen::Matrix<double, 42, 1>* f_ext, Eigen::Vector3d* lambda_tilde, Eigen::Vector3d* lambda_i, Eigen::SparseMatrix<double>* Pc_hat, Eigen::Matrix<double, 42, 1>* v_g_old, double h_com1, Eigen::Vector3d* Drone_Interaction) {
	// updates the interface jacobian
	Interface_Jacobian(DP, DDP, R_vec, R_tilde_mat, phin_list, Ai, Ai_old, Ai_dot, h);
	(*dot_phin_list) = (*Ai) * (*v_g);
	// updates the effective mass matrix
	(*M_tilde_inv) = (*Ai) * (*IPMi) * (*Ai).transpose();
	(*M_tilde_inv) += (*M_temp_offset);
	(*M_tilde) = (*M_tilde_inv).inverse();
	(*M_tilde_inv) -= (*M_temp_offset);

	// updates the external force vector
	(*f_ext)(0,0) = (*Drag)(0,0) + (*Gravity)(0,0) + (*Drone_Interaction)(0,0);
	(*f_ext)(1,0) = (*Drag)(1,0) + (*Gravity)(1,0) + (*Drone_Interaction)(1,0);
	(*f_ext)(2,0) = (*Drag)(2,0) + (*Gravity)(2,0) + (*Drone_Interaction)(2,0);
	(*lambda_tilde) = (*M_tilde) * ((*Ai) * (*IPMi) * (*f_ext) + (*Ai_dot) * (*v_g) + (*Ai) * (*Pc_hat) * (((*v_g) - (*v_g_old)) / h_com1));
	(*v_g_old) = (*v_g);
}

// updates the interface velocities and positions
void BRIM::brim::bmn_update(double* r_type, Eigen::Vector3d* phin_list, Eigen::Vector3d* dot_phin_list, Eigen::Vector3d* lambda_tilde, Eigen::Vector3d* lambda_i, Eigen::Vector3d* R_vec_est, Eigen::Matrix3d* M_tilde_inv, double h, Eigen::Vector3d* dx_lims, Eigen::Vector3d* x_lims) {
	// if using ZOH then do nothing
	if ((*r_type) == 0) {
		// nothing
	}
	// if using FOH then update the phin with the dot_phin
	else if ((*r_type) == 1) {
		BRIM::brim::RK4_vec_update(phin_list, *dot_phin_list, h);
	}
	// if using RIM then use the projected matrices to update the dot_phin and then phin
	else if ((*r_type) == 2) {
		Eigen::Vector3d temp = Eigen::Vector3d::Zero();
		//std::cout << "lambda_tilde: " << (*lambda_tilde)[0] << ", " << (*lambda_tilde)[1] << ", " << (*lambda_tilde)[2] << std::endl;
		//std::cout << "lambda_i: " << (*lambda_i)[0] << ", " << (*lambda_i)[1] << ", " << (*lambda_i)[2] << std::endl;
		//std::cout << "M_tilde_inv: " << (*M_tilde_inv)(0,0) << ", " << (*M_tilde_inv)(0,1) << ", " << (*M_tilde_inv)(0,2) << std::endl;
		temp = (*M_tilde_inv) * ((*lambda_tilde) + (*lambda_i));
		//temp = (*M_tilde_inv) * ((*lambda_tilde));
		//std::cout << "temp: " << temp[0] << ", " << temp[1] << ", " << temp[2] << std::endl;
		BRIM::brim::RK4_vec_update(dot_phin_list, temp, h);
		BRIM::brim::Vec3Lim(dot_phin_list, dx_lims);
		BRIM::brim::RK4_vec_update(phin_list, *dot_phin_list, h);
		BRIM::brim::Vec3Lim(phin_list, x_lims);
	}
	// R_vec is equivalent to the phin
	(*R_vec_est)[0] = (*phin_list)[0];
	(*R_vec_est)[1] = (*phin_list)[1];
	(*R_vec_est)[2] = (*phin_list)[2];
}

// helper function to fill values in a sparse matrix with a given 3by3 denses matrix
void BRIM::brim::sparse_fill(Eigen::SparseMatrix<double>* tb_filled, Eigen::Matrix3d* t_fill, int row, int col) {
	for (int i = 0; i < (*t_fill).rows(); i++) {
		(*tb_filled).insert(row + i, col) = (*t_fill)(i, 0);
		(*tb_filled).insert(row + i, col + 1) = (*t_fill)(i, 1);
		(*tb_filled).insert(row + i, col + 2) = (*t_fill)(i, 2);
	}
}

// helper function to replace values in a sparse matrix with a given 3by3 denses matrix
void BRIM::brim::sparse_replace(Eigen::SparseMatrix<double>* tb_replaced, Eigen::Matrix3d* t_replace, int row, int col) {
	for (int i = 0; i < (*t_replace).rows(); i++) {
		(*tb_replaced).coeffRef(row + i, col) = (*t_replace)(i, 0);
		(*tb_replaced).coeffRef(row + i, col + 1) = (*t_replace)(i, 1);
		(*tb_replaced).coeffRef(row + i, col + 2) = (*t_replace)(i, 2);
	}
}

// helper function for RK4_vec_update
void BRIM::brim::RK4_update(double* xn, double* xn_dot, double* h) {
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
void BRIM::brim::RK4_vec_update(Eigen::Vector3d* xn, Eigen::Vector3d xn_dot, double h) {
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

// function to limit phin or dot phin vector with given limits
void BRIM::brim::Vec3Lim(Eigen::Vector3d* vec, Eigen::Vector3d* lim) {
	for (int i = 0; i < (*vec).rows(); i++) {
		if ((*vec)(i) > (*lim)(i)) {
			(*vec)(i) = (*lim)(i);
		}
		else if ((*vec)(i) < -(*lim)(i)) {
			(*vec)(i) = -(*lim)(i);
		}
	}
}

// function to convert a ros msg to a sparse matrix
void BRIM::brim::msg_to_matrix(std_msgs::msg::Float64MultiArray min, Eigen::SparseMatrix<double>* mout) {
	int rows = min.data[0];
	int cols = min.data[1];
	int size = min.data[2];
	float data = 0.0;
	int i,j = 0;
	std::vector< Eigen::Triplet<double> > tripletList;
	tripletList.reserve(size-1);
	for (int k = 1; k < size; k++) {
		data = min.data[k * 3 + 2];
		i = min.data[k * 3 + 0];
		j = min.data[k * 3 + 1];
		if (data != 0) {
			tripletList.push_back(Eigen::Triplet<double>(i, j, data));
		}
	}
	(*mout).setFromTriplets(tripletList.begin(), tripletList.end());
}