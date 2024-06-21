#include "../include/rb_quad_sim/rb_helpers.h"

// plane interaction functions
// helper function to calculate the distance between the drone and a plane
void RBH::plane_interface_distance(Eigen::Vector3d* object_pos, Eigen::Matrix <double, 6, 1>* phin_list, planes* plane_eqns, double object_con_rad) {
	Eigen::Vector3d pos;
	double numerator;
	double denominator;
	double ans;
	Eigen::Vector3d nhatc;
	// loop through all 6 planes
	for (int i = 0; i < 6; i++) {
		// find the relevant unit normal vector
		nhatc = (*plane_eqns).nhat.row(i);
		// find the position of the drone relative to the plane
		pos = (*object_pos) + (nhatc * -object_con_rad);
		// calculate the absolute distance between the drone and the plane
		numerator = ((*plane_eqns).n.row(i)*pos) - (*plane_eqns).d[i];
		// find the relevant normal vector
		denominator = (*plane_eqns).n.row(i).norm();
		// calculate the distance between the drone and the plane
		ans = numerator / denominator;
		(*phin_list)[i] = ans;
	}
}

// helper function to calculate the velocity of the drone relative to a plane
void RBH::plane_interface_velocity(Eigen::SparseMatrix<double>* An_mat, Eigen::SparseMatrix<double>* global_vels, Eigen::Matrix <double, 6, 1>* dphin_list) {
	Eigen::SparseMatrix<double> temp;
	// using the constraint jacobian and system velocities to find the relative velocity between the drone and the plane
	temp = (*An_mat) * (*global_vels).transpose();
	(*dphin_list) = temp.toDense();
}

// helper function to calculate the interaction jacobian between the drone and a plane
void RBH::plane_interface_jacobian(Eigen::SparseMatrix<double>* An_mat, planes* plane_eqns) {
	int col1 = 0;
	int col2 = 0;
	int col3 = 0;
	// use the plane equation normal vectors to fill in the interface jacobian
	for (int i = 0; i < 6; i++) {
		(*An_mat).coeffRef(i, 0) = (*plane_eqns).n(i,0);
		(*An_mat).coeffRef(i, 1) = (*plane_eqns).n(i,1);
		(*An_mat).coeffRef(i, 2) = (*plane_eqns).n(i,2);
		col1 = (i * 6) + 6;
		col2 = (i * 6) + 7;
		col3 = (i * 6) + 8;
		(*An_mat).coeffRef(i, col1) = -1 * (*plane_eqns).n(i,0);
		(*An_mat).coeffRef(i, col2) = -1 * (*plane_eqns).n(i,1);
		(*An_mat).coeffRef(i, col3) = -1 * (*plane_eqns).n(i,2);
	}
	//std::cout << "An_mat: " << (*An_mat).block(0,0,6,42) << std::endl;
}

// Matrix functions
// helper function to calulate the update mass matrix
void RBH::massMatrix(Eigen::SparseMatrix<double>* M_mat, Eigen::SparseMatrix<double>* M_mat_star, Eigen::SparseMatrix<double>* M_mat_inv, Eigen::SparseMatrix<double>* M_mat_dot, Eigen::SparseMatrix<double>* M_mat_pre, Eigen::SparseMatrix<double>* N, Quadcopter::UAS* drone, double step_size) {
	// find the mass matrix
	(*M_mat_pre) = (*M_mat);
	SmassMatrix(M_mat_star, drone);
	//std::cout << "M_mat_star: " << (*M_mat_star).block(0,0,42,42) << std::endl;
	(*M_mat) = (*N).transpose() * (*M_mat_star) * (*N);
	//std::cout << "M_mat: " << (*M_mat).block(0,0,42,42) << std::endl;
	// find the derivative of the mass matrix
	(*M_mat_dot) = ((*M_mat) - (*M_mat_pre)) / step_size;
	//std::cout << "M_mat_dot: " << (*M_mat_dot).block(0,0,42,42) << std::endl;
	// find the inverse of the mass matrix by parts for simplicity
	//M_mat_inv = M_mat.inverse();
	Eigen::Matrix3d dm = (*M_mat).block(0, 0, 3, 3);
	Eigen::Matrix3d dI = (*M_mat).block(3, 3, 3, 3);
	//std::cout << "dI: " << dI << std::endl;
	Eigen::Matrix3d dmi = dm.inverse();
	Eigen::Matrix3d dIi = dI.inverse();
	//std::cout << "dIi: " << dIi << std::endl;
	sparse_replace(M_mat_inv, &dmi, 0, 0);
	sparse_replace(M_mat_inv, &dIi, 3, 3);
	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
	for (int i = 2; i < 14; i++) {
		sparse_replace(M_mat_inv, &I, (3 * i), (3 * i));
	}
	//std::cout << "M_mat_inv: " << (*M_mat_inv).block(0,0,42,42) << std::endl;
}

// helper function to calulate the updated mass star matrix
void RBH::SmassMatrix(Eigen::SparseMatrix<double>* M_mat_star, Quadcopter::UAS* drone) {
	// calculate the mass portion of the mass matrix
	Eigen::Matrix3d mass = (*drone).get_state().mass * Eigen::Matrix3d::Identity();
	// calculate the inertia portion of the mass matrix
	Eigen::Matrix4d Iars;
	Smasswork(&Iars, drone);
	
	// fill in the sparse matrix
	sparse_replace(M_mat_star, &mass, 0, 0);
	sparse_replace4(M_mat_star, &Iars, 3, 3);
}

// helper to complete the work behind the mass star matrix
void RBH::Smasswork(Eigen::Matrix4d* Iars, Quadcopter::UAS* drone) {
	// find the relevant values
	double w = (*drone).get_state().g_att.w();
	double x = (*drone).get_state().g_att.x();
	double y = (*drone).get_state().g_att.y();
	double z = (*drone).get_state().g_att.z();
	double Ixx = (*drone).get_state().I(0, 0);
	double Iyy = (*drone).get_state().I(1, 1);
	double Izz = (*drone).get_state().I(2, 2);

	//std::cout << "Drone state: " << w << ", " << x << ", " << y << ", " << z << ", " << Ixx << ", " << Iyy << ", " << Izz << std::endl;

	// calculate some constants
	double qsqnorm = std::pow(w,2) + std::pow(x,2) + std::pow(y,2) + std::pow(z,2);
	double qsqnorm4 = std::pow(qsqnorm,4) * 4;

	//std::cout << "qsqnorm: " << qsqnorm << std::endl;
	//std::cout << "qsqnorm4: " << qsqnorm4 << std::endl;

	// calculate the adjusted inertia matrix
	(*Iars)(0,0) = (Ixx*std::pow(x,2) + Iyy*std::pow(y,2) + Izz*std::pow(z,2))/(qsqnorm4);
	(*Iars)(0,1) = -(Ixx*w*x - Iyy*y*z + Izz*y*z)/(qsqnorm4);
	(*Iars)(0,2) = -(Iyy*w*y + Ixx*x*z - Izz*x*z)/(qsqnorm4);
	(*Iars)(0,3) = -(Iyy*x*y - Ixx*x*y + Izz*w*z)/(qsqnorm4);
	(*Iars)(1,0) = -(Ixx*w*x - Iyy*y*z + Izz*y*z)/(qsqnorm4);
	(*Iars)(1,1) = (Ixx*std::pow(w,2)+ Izz*std::pow(y,2) + Iyy*std::pow(z,2))/(qsqnorm4);
	(*Iars)(1,2) = -(Iyy*w*z - Ixx*w*z + Izz*x*y)/(qsqnorm4);
	(*Iars)(1,3) = -(Ixx*w*y - Izz*w*y + Iyy*x*z)/(qsqnorm4);
	(*Iars)(2,0) = -(Iyy*w*y + Ixx*x*z - Izz*x*z)/(qsqnorm4);
	(*Iars)(2,1) = -(Iyy*w*z - Ixx*w*z + Izz*x*y)/(qsqnorm4);
	(*Iars)(2,2) = (Iyy*std::pow(w,2)+ Izz*std::pow(x,2)+ Ixx*std::pow(z,2))/(qsqnorm4);
	(*Iars)(2,3) = -(Izz*w*x - Iyy*w*x + Ixx*y*z)/(qsqnorm4);
	(*Iars)(3,0) = -(Iyy*x*y - Ixx*x*y + Izz*w*z)/(qsqnorm4);
	(*Iars)(3,1) = -(Ixx*w*y - Izz*w*y + Iyy*x*z)/(qsqnorm4);
	(*Iars)(3,2) = -(Izz*w*x - Iyy*w*x + Ixx*y*z)/(qsqnorm4);
	(*Iars)(3,3) = (Izz*std::pow(w,2)+ Iyy*std::pow(x,2)+ Ixx*std::pow(y,2))/(qsqnorm4);
}

// helper function to calulate the updated omega matrix
void RBH::omegaMatrix(Eigen::SparseMatrix<double>* O_mat, Eigen::SparseMatrix<double>* O_mat_star, Eigen::SparseMatrix<double>* N_mat, Eigen::SparseMatrix<double>* N_mat_dot, Eigen::SparseMatrix<double>* M_mat_star, Quadcopter::UAS* drone) {
	const int size = 42;
	// find the omega star matrix
	SomegaMatrix(O_mat_star, drone);
	// find the omega matrix using the star matrix, the star mass matrix, and the transformation matrices
	Eigen::SparseMatrix <double> pt1(size, size);
	Eigen::SparseMatrix <double> pt2(size, size);
	pt1 = (*N_mat_dot).transpose() * (*M_mat_star) * (*N_mat);
	pt2 = (*N_mat).transpose() * (*O_mat_star) * (*N_mat);
	(*O_mat) = pt1 + pt2;
}

// helper function to calulate the updated omega star matrix
void RBH::SomegaMatrix(Eigen::SparseMatrix<double>* O_mat_star, Quadcopter::UAS* drone) {
	// find the needed values
	Eigen::Quaterniond qdot = (*drone).get_state().g_datt;
	Eigen::Quaterniond q = (*drone).get_state().g_att;
	Eigen::Matrix3d I = (*drone).get_state().I;
	// find the omega star matrix
	Eigen::Matrix4d omegastarwork = Somegawork(&qdot, &I, &q);
	// fill in the sparse matrix
	sparse_replace4(O_mat_star, &omegastarwork, 3, 3);
}

// helper function to calculate the block matrices to be added to the omega star matrix
Eigen::Matrix4d RBH::Somegawork(Eigen::Quaterniond* qdot, Eigen::Matrix3d* I, Eigen::Quaterniond* q) {
	// find the needed values
	Eigen::Matrix4d out;
	double w = (*q).w();
	double x = (*q).x();
	double y = (*q).y();
	double z = (*q).z();
	double dw = (*qdot).w();
	double dx = (*qdot).x();
	double dy = (*qdot).y();
	double dz = (*qdot).z();
	double Ixx = (*I)(0, 0);
	double Iyy = (*I)(1, 1);
	double Izz = (*I)(2, 2);

	// calculate some constants
	double qsqnorm = std::pow(w,2) + std::pow(x,2) + std::pow(y,2) + std::pow(z,2);
	double qsqnorm4 = std::pow(qsqnorm,4) * 4;

	// calculate the block matrices for omega star
	out(0,0) = (Ixx*dx*std::pow(x,3) + Iyy*dy*std::pow(y,3) + Izz*dz*std::pow(z,3) + 8*Ixx*dw*w*std::pow(x,2) - 7*Ixx*dx*std::pow(w,2)*x + 8*Iyy*dw*w*std::pow(y,2) - 7*Iyy*dy*std::pow(w,2)*y + Ixx*dx*x*std::pow(y,2) + 8*Izz*dw*w*std::pow(z,2) + Iyy*dy*std::pow(x,2)*y - 7*Izz*dz*std::pow(w,2)*z + Ixx*dx*x*std::pow(z,2) + Izz*dz*std::pow(x,2)*z + Iyy*dy*y*std::pow(z,2) + Izz*dz*std::pow(y,2)*z + 8*Ixx*dz*w*x*y - 8*Iyy*dz*w*x*y - 8*Ixx*dy*w*x*z + 8*Izz*dy*w*x*z + 8*Iyy*dx*w*y*z - 8*Izz*dx*w*y*z)/(8*std::pow(qsqnorm,5));
	out(0,1) = (dw*(Ixx*std::pow(x,3) - 7*Ixx*std::pow(w,2)*x + Ixx*x*std::pow(y,2) + Ixx*x*std::pow(z,2) + 8*Iyy*w*y*z - 8*Izz*w*y*z))/(8*std::pow(qsqnorm,5)) - (dy*((2*w*(Iyy*w*z - Ixx*w*z + Izz*x*y))/std::pow(qsqnorm,5) + (z*(Ixx - Iyy))/(qsqnorm4)))/2 - (dz*((2*w*(Ixx*w*y - Izz*w*y + Iyy*x*z))/std::pow(qsqnorm,5) - (y*(Ixx - Izz))/(qsqnorm4)))/2 + (dx*w*(3*Ixx*std::pow(w,2) - Ixx*std::pow(x,2) - Ixx*std::pow(y,2) + 4*Izz*std::pow(y,2) - Ixx*std::pow(z,2) + 4*Iyy*std::pow(z,2)))/(4*std::pow(qsqnorm,5));
	out(0,2) = (dw*(Iyy*std::pow(y,3) - 7*Iyy*std::pow(w,2)*y + Iyy*std::pow(x,2)*y + Iyy*y*std::pow(z,2) - 8*Ixx*w*x*z + 8*Izz*w*x*z))/(8*std::pow(qsqnorm,5)) - (dx*((2*w*(Iyy*w*z - Ixx*w*z + Izz*x*y))/std::pow(qsqnorm,5) + (z*(Ixx - Iyy))/(qsqnorm4)))/2 - (dz*((2*w*(Izz*w*x - Iyy*w*x + Ixx*y*z))/std::pow(qsqnorm,5) + (x*(Iyy - Izz))/(qsqnorm4)))/2 + (dy*w*(3*Iyy*std::pow(w,2) - Iyy*std::pow(x,2) + 4*Izz*std::pow(x,2) - Iyy*std::pow(y,2) + 4*Ixx*std::pow(z,2) - Iyy*std::pow(z,2)))/(4*std::pow(qsqnorm,5));
	out(0,3) = (dw*(Izz*std::pow(z,3) - 7*Izz*std::pow(w,2)*z + Izz*std::pow(x,2)*z + Izz*std::pow(y,2)*z + 8*Ixx*w*x*y - 8*Iyy*w*x*y))/(8*std::pow(qsqnorm,5)) - (dx*((2*w*(Ixx*w*y - Izz*w*y + Iyy*x*z))/std::pow(qsqnorm,5) - (y*(Ixx - Izz))/(qsqnorm4)))/2 - (dy*((2*w*(Izz*w*x - Iyy*w*x + Ixx*y*z))/std::pow(qsqnorm,5) + (x*(Iyy - Izz))/(qsqnorm4)))/2 + (dz*w*(3*Izz*std::pow(w,2) + 4*Iyy*std::pow(x,2) - Izz*std::pow(x,2) + 4*Ixx*std::pow(y,2) - Izz*std::pow(y,2) - Izz*std::pow(z,2)))/(4*std::pow(qsqnorm,5));
	out(1,0) = (dx*(Ixx*std::pow(w,3) - 7*Ixx*w*std::pow(x,2) + Ixx*w*std::pow(y,2) + Ixx*w*std::pow(z,2) + 8*Iyy*x*y*z - 8*Izz*x*y*z))/(8*std::pow(qsqnorm,5)) - (dy*((2*x*(Iyy*w*y + Ixx*x*z - Izz*x*z))/std::pow(qsqnorm,5) - (z*(Ixx - Izz))/(qsqnorm4)))/2 - (dz*((2*x*(Iyy*x*y - Ixx*x*y + Izz*w*z))/std::pow(qsqnorm,5) + (y*(Ixx - Iyy))/(qsqnorm4)))/2 - (dw*x*(Ixx*std::pow(w,2) - 3*Ixx*std::pow(x,2) + Ixx*std::pow(y,2) - 4*Iyy*std::pow(y,2) + Ixx*std::pow(z,2) - 4*Izz*std::pow(z,2)))/(4*std::pow(qsqnorm,5));
	out(1,1) = (Ixx*dw*std::pow(w,3) + Izz*dy*std::pow(y,3) + Iyy*dz*std::pow(y,3) - 7*Ixx*dw*w*std::pow(x,2) + 8*Ixx*dx*std::pow(w,2)*x + Ixx*dw*w*std::pow(y,2) + Izz*dy*std::pow(w,2)*y + Ixx*dw*w*std::pow(z,2) + 8*Izz*dx*x*std::pow(y,2) + Iyy*dz*std::pow(w,2)*z - 7*Izz*dy*std::pow(x,2)*y + 8*Iyy*dx*x*std::pow(z,2) - 7*Iyy*dz*std::pow(x,2)*z + Iyy*dz*std::pow(y,2)*z + Izz*dy*y*std::pow(z,2) - 8*Ixx*dz*w*x*y + 8*Izz*dz*w*x*y + 8*Ixx*dy*w*x*z - 8*Iyy*dy*w*x*z + 8*Iyy*dw*x*y*z - 8*Izz*dw*x*y*z)/(8*std::pow(qsqnorm,5));
	out(1,2) = (dx*(Izz*std::pow(y,3) + Izz*std::pow(w,2)*y - 7*Izz*std::pow(x,2)*y + Izz*y*std::pow(z,2) + 8*Ixx*w*x*z - 8*Iyy*w*x*z))/(8*std::pow(qsqnorm,5)) - (dw*((2*x*(Iyy*w*y + Ixx*x*z - Izz*x*z))/std::pow(qsqnorm,5) - (z*(Ixx - Izz))/(qsqnorm4)))/2 - (dz*((2*x*(Izz*w*x - Iyy*w*x + Ixx*y*z))/std::pow(qsqnorm,5) + (w*(Iyy - Izz))/(qsqnorm4)))/2 + (dy*x*(4*Iyy*std::pow(w,2) - Izz*std::pow(w,2) + 3*Izz*std::pow(x,2) - Izz*std::pow(y,2) + 4*Ixx*std::pow(z,2) - Izz*std::pow(z,2)))/(4*std::pow(qsqnorm,5));
	out(1,3) = (dx*(Iyy*std::pow(z,3) + Iyy*std::pow(w,2)*z - 7*Iyy*std::pow(x,2)*z + Iyy*std::pow(y,2)*z - 8*Ixx*w*x*y + 8*Izz*w*x*y))/(8*std::pow(qsqnorm,5)) - (dw*((2*x*(Iyy*x*y - Ixx*x*y + Izz*w*z))/std::pow(qsqnorm,5) + (y*(Ixx - Iyy))/(qsqnorm4)))/2 - (dy*((2*x*(Izz*w*x - Iyy*w*x + Ixx*y*z))/std::pow(qsqnorm,5) + (w*(Iyy - Izz))/(qsqnorm4)))/2 - (dz*x*(Iyy*std::pow(w,2) - 4*Izz*std::pow(w,2) - 3*Iyy*std::pow(x,2) - 4*Ixx*std::pow(y,2) + Iyy*std::pow(y,2) + Iyy*std::pow(z,2)))/(4*std::pow(qsqnorm,5));
    out(2,0) = (dy*(Iyy*std::pow(w,3) + Iyy*w*std::pow(x,2) - 7*Iyy*w*std::pow(y,2) + Iyy*w*std::pow(z,2) - 8*Ixx*x*y*z + 8*Izz*x*y*z))/(8*std::pow(qsqnorm,5)) - (dx*((2*y*(Ixx*w*x - Iyy*y*z + Izz*y*z))/std::pow(qsqnorm,5) + (z*(Iyy - Izz))/(qsqnorm4)))/2 - (dz*((2*y*(Iyy*x*y - Ixx*x*y + Izz*w*z))/std::pow(qsqnorm,5) + (x*(Ixx - Iyy))/(qsqnorm4)))/2 - (dw*y*(Iyy*std::pow(w,2) - 4*Ixx*std::pow(x,2) + Iyy*std::pow(x,2) - 3*Iyy*std::pow(y,2) + Iyy*std::pow(z,2) - 4*Izz*std::pow(z,2)))/(4*std::pow(qsqnorm,5));
	out(2,1) = (dy*(Izz*std::pow(x,3) + Izz*std::pow(w,2)*x - 7*Izz*x*std::pow(y,2) + Izz*x*std::pow(z,2) + 8*Ixx*w*y*z - 8*Iyy*w*y*z))/(8*std::pow(qsqnorm,5)) - (dw*((2*y*(Ixx*w*x - Iyy*y*z + Izz*y*z))/std::pow(qsqnorm,5) + (z*(Iyy - Izz))/(qsqnorm4)))/2 - (dz*((2*y*(Ixx*w*y - Izz*w*y + Iyy*x*z))/std::pow(qsqnorm,5) - (w*(Ixx - Izz))/(qsqnorm4)))/2 + (dx*y*(4*Ixx*std::pow(w,2) - Izz*std::pow(w,2) - Izz*std::pow(x,2) + 3*Izz*std::pow(y,2) + 4*Iyy*std::pow(z,2) - Izz*std::pow(z,2)))/(4*std::pow(qsqnorm,5));
	out(2,2) = (Iyy*dw*std::pow(w,3) + Izz*dx*std::pow(x,3) + Ixx*dz*std::pow(z,3) + Iyy*dw*w*std::pow(x,2) + Izz*dx*std::pow(w,2)*x - 7*Iyy*dw*w*std::pow(y,2) + 8*Iyy*dy*std::pow(w,2)*y + Iyy*dw*w*std::pow(z,2) + Ixx*dz*std::pow(w,2)*z - 7*Izz*dx*x*std::pow(y,2) + 8*Izz*dy*std::pow(x,2)*y + Ixx*dz*std::pow(x,2)*z + Izz*dx*x*std::pow(z,2) + 8*Ixx*dy*y*std::pow(z,2) - 7*Ixx*dz*std::pow(y,2)*z + 8*Iyy*dz*w*x*y - 8*Izz*dz*w*x*y + 8*Ixx*dx*w*y*z - 8*Iyy*dx*w*y*z - 8*Ixx*dw*x*y*z + 8*Izz*dw*x*y*z)/(8*std::pow(qsqnorm,5));
	out(2,3) = (dy*(Ixx*std::pow(z,3) + Ixx*std::pow(w,2)*z + Ixx*std::pow(x,2)*z - 7*Ixx*std::pow(y,2)*z + 8*Iyy*w*x*y - 8*Izz*w*x*y))/(8*std::pow(qsqnorm,5)) - (dw*((2*y*(Iyy*x*y - Ixx*x*y + Izz*w*z))/std::pow(qsqnorm,5) + (x*(Ixx - Iyy))/(qsqnorm4)))/2 - (dx*((2*y*(Ixx*w*y - Izz*w*y + Iyy*x*z))/std::pow(qsqnorm,5) - (w*(Ixx - Izz))/(qsqnorm4)))/2 - (dz*y*(Ixx*std::pow(w,2) - 4*Izz*std::pow(w,2) + Ixx*std::pow(x,2) - 4*Iyy*std::pow(x,2) - 3*Ixx*std::pow(y,2) + Ixx*std::pow(z,2)))/(4*std::pow(qsqnorm,5));
	out(3,0) = (dz*(Izz*std::pow(w,3) + Izz*w*std::pow(x,2) + Izz*w*std::pow(y,2) - 7*Izz*w*std::pow(z,2) + 8*Ixx*x*y*z - 8*Iyy*x*y*z))/(8*std::pow(qsqnorm,5)) - (dx*((2*z*(Ixx*w*x - Iyy*y*z + Izz*y*z))/std::pow(qsqnorm,5) + (y*(Iyy - Izz))/(qsqnorm4)))/2 - (dy*((2*z*(Iyy*w*y + Ixx*x*z - Izz*x*z))/std::pow(qsqnorm,5) - (x*(Ixx - Izz))/(qsqnorm4)))/2 - (dw*z*(Izz*std::pow(w,2) - 4*Ixx*std::pow(x,2) + Izz*std::pow(x,2) - 4*Iyy*std::pow(y,2) + Izz*std::pow(y,2) - 3*Izz*std::pow(z,2)))/(4*std::pow(qsqnorm,5));
	out(3,1) = (dz*(Iyy*std::pow(x,3) + Iyy*std::pow(w,2)*x + Iyy*x*std::pow(y,2) - 7*Iyy*x*std::pow(z,2) - 8*Ixx*w*y*z + 8*Izz*w*y*z))/(8*std::pow(qsqnorm,5)) - (dw*((2*z*(Ixx*w*x - Iyy*y*z + Izz*y*z))/std::pow(qsqnorm,5) + (y*(Iyy - Izz))/(qsqnorm4)))/2 - (dy*((2*z*(Iyy*w*z - Ixx*w*z + Izz*x*y))/std::pow(qsqnorm,5) + (w*(Ixx - Iyy))/(qsqnorm4)))/2 + (dx*z*(4*Ixx*std::pow(w,2) - Iyy*std::pow(w,2) - Iyy*std::pow(x,2) - Iyy*std::pow(y,2) + 4*Izz*std::pow(y,2) + 3*Iyy*std::pow(z,2)))/(4*std::pow(qsqnorm,5));
	out(3,2) = (dz*(Ixx*std::pow(y,3) + Ixx*std::pow(w,2)*y + Ixx*std::pow(x,2)*y - 7*Ixx*y*std::pow(z,2) + 8*Iyy*w*x*z - 8*Izz*w*x*z))/(8*std::pow(qsqnorm,5)) - (dw*((2*z*(Iyy*w*y + Ixx*x*z - Izz*x*z))/std::pow(qsqnorm,5) - (x*(Ixx - Izz))/(qsqnorm4)))/2 - (dx*((2*z*(Iyy*w*z - Ixx*w*z + Izz*x*y))/std::pow(qsqnorm,5) + (w*(Ixx - Iyy))/(qsqnorm4)))/2 - (dy*z*(Ixx*std::pow(w,2) - 4*Iyy*std::pow(w,2) + Ixx*std::pow(x,2) - 4*Izz*std::pow(x,2) + Ixx*std::pow(y,2) - 3*Ixx*std::pow(z,2)))/(4*std::pow(qsqnorm,5));                                                               
	out(3,3) = (Izz*dw*std::pow(w,3) + Iyy*dx*std::pow(x,3) + Ixx*dy*std::pow(y,3) + Iyy*dx*std::pow(w,2)*x + Izz*dw*w*std::pow(x,2) + Ixx*dy*std::pow(w,2)*y + Izz*dw*w*std::pow(y,2) + Ixx*dy*std::pow(x,2)*y + Iyy*dx*x*std::pow(y,2) - 7*Izz*dw*w*std::pow(z,2) + 8*Izz*dz*std::pow(w,2)*z - 7*Iyy*dx*x*std::pow(z,2) + 8*Iyy*dz*std::pow(x,2)*z - 7*Ixx*dy*y*std::pow(z,2) + 8*Ixx*dz*std::pow(y,2)*z + 8*Iyy*dy*w*x*z - 8*Izz*dy*w*x*z - 8*Ixx*dx*w*y*z + 8*Izz*dx*w*y*z + 8*Ixx*dw*x*y*z - 8*Iyy*dw*x*y*z)/(8*std::pow(qsqnorm,5));
	
	return out;
}

// helper function to calculate the transformation matrix from global quaternion form to global angular velocity form
void RBH::transformMatrix(Eigen::SparseMatrix<double>* N_mat, Quadcopter::UAS* drone) {
	// fill in the basic parts
    Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
	for (int i = 0; i < 7; i++) {
		sparse_replace(N_mat, &I3, (7 * i), (6 * i));
	}
	//std::cout << "N_mat: " << (*N_mat).block(0,0,42,42) << std::endl;
	// fill in the quaternion part
	Eigen::Matrix<double, 4, 3> ET;
	Eigen::Vector4d qvec = { (*drone).get_state().g_att.w(), (*drone).get_state().g_att.x(), (*drone).get_state().g_att.y(), (*drone).get_state().g_att.z() };
	//std::cout << "qvec: " << qvec[0] << ", " << qvec[1] << ", " << qvec[2] << ", " << qvec[3] << std::endl;
	ET << -qvec[1], -qvec[2], -qvec[3],
		 qvec[0], qvec[3], -qvec[2],
		 -qvec[3], qvec[0], qvec[1],
		 qvec[2], -qvec[1], qvec[0];
	Eigen::Matrix<double, 4, 3> Ehat = 0.5 * ET;
	Eigen::Matrix3d temp = Ehat.block(0, 0, 3, 3);
	//std::cout << "Ehat: " << Ehat << std::endl;
	sparse_replace(N_mat, &temp, 3, 3);
	N_mat->coeffRef(6,3) = Ehat(3,0);
	N_mat->coeffRef(6,4) = Ehat(3,1);
	N_mat->coeffRef(6,5) = Ehat(3,2);
	//std::cout << "N_mat: " << (*N_mat).block(0,0,42,42) << std::endl;
}

// helper function to calculate the derivative of the transformation matrix
void RBH::dotTransform(Eigen::SparseMatrix<double>* N_mat_dot, Quadcopter::UAS* drone) {
	// use the skew symmetric  matrix of the angular velocity to calculate the derivative of the rotation matrix
	Eigen::Matrix3d dravelskew;
	dravelskew << 0, -(*drone).get_state().b_avel[2], (*drone).get_state().b_avel[1],
				  (*drone).get_state().b_avel[2], 0, -(*drone).get_state().b_avel[0],
				  -(*drone).get_state().b_avel[1], (*drone).get_state().b_avel[0], 0;
	Eigen::Matrix<double, 3, 4> G;
	Eigen::Vector4d qvec = { (*drone).get_state().g_att.w(), (*drone).get_state().g_att.x(), (*drone).get_state().g_att.y(), (*drone).get_state().g_att.z() };
	G << -qvec[1], qvec[0], qvec[3], -qvec[2],
		 -qvec[2], -qvec[3], qvec[0], qvec[1],
		 -qvec[3], qvec[2], -qvec[1], qvec[0];
	Eigen::Matrix3d RB = (*drone).get_state().r_mat;
	Eigen::Matrix3d RBdot = RB * dravelskew;
	// calculate the derivate of the E term
	Eigen::Matrix<double, 3, 4> Gdiff = (2*G) - (G*((G.transpose()*G).inverse()));
	Eigen::Matrix<double, 4, 3> Ehatdot = 0.5 * (Gdiff.transpose());
	Eigen::Matrix3d temp = Ehatdot.block(0, 0, 3, 3);
	sparse_replace(N_mat_dot, &temp, 3, 3);
	N_mat_dot->coeffRef(6,3) = Ehatdot(3,0);
	N_mat_dot->coeffRef(6,4) = Ehatdot(3,1);
	N_mat_dot->coeffRef(6,5) = Ehatdot(3,2);
}

// helper function to calculate the derivative of the interaction jacobian matrix for a plane
void RBH::dotJacobian(Eigen::SparseMatrix<double>* An_mat_dot, planes* plane_eqns) {
	Eigen::Vector3d An1;
	// use the plane normal vectors to fill in the derivative of the interaction jacobian matrix
	for (int i = 0; i < 6; i++) {
		(*An_mat_dot).coeffRef(i, 0) = (*plane_eqns).n(i,0);
		(*An_mat_dot).coeffRef(i, 1) = (*plane_eqns).n(i,1);
		(*An_mat_dot).coeffRef(i, 2) = (*plane_eqns).n(i,2);
		An1 = -1 * (*plane_eqns).n.row(i);
		(*An_mat_dot).coeffRef(i, (i * 6) + 6) = An1[0];
		(*An_mat_dot).coeffRef(i, (i * 6) + 7) = An1[1];
		(*An_mat_dot).coeffRef(i, (i * 6) + 8) = An1[2];
	}
}

// helper function to fill values in a sparse matrix with a given 3by3 denses matrix
void RBH::sparse_fill(Eigen::SparseMatrix<double>* tb_filled, Eigen::Matrix3d* t_fill, int row, int col) {
	for (int i = 0; i < (*t_fill).rows(); i++) {
		(*tb_filled).insert(row + i, col) = (*t_fill)(i, 0);
		(*tb_filled).insert(row + i, col + 1) = (*t_fill)(i, 1);
		(*tb_filled).insert(row + i, col + 2) = (*t_fill)(i, 2);
	}
}

// helper function to replace values in a sparse matrix with a given 3by3 denses matrix
void RBH::sparse_replace(Eigen::SparseMatrix<double>* tb_replaced, Eigen::Matrix3d* t_replace, int row, int col) {
	for (int i = 0; i < (*t_replace).rows(); i++) {
		(*tb_replaced).coeffRef(row + i, col) = (*t_replace)(i, 0);
		(*tb_replaced).coeffRef(row + i, col + 1) = (*t_replace)(i, 1);
		(*tb_replaced).coeffRef(row + i, col + 2) = (*t_replace)(i, 2);
	}
}

// helper function to fill values in a sparse matrix with a given 3by3 denses matrix
void RBH::sparse_fill4(Eigen::SparseMatrix<double>* tb_filled, Eigen::Matrix4d* t_fill, int row, int col) {
	for (int i = 0; i < (*t_fill).rows(); i++) {
		(*tb_filled).insert(row + i, col) = (*t_fill)(i, 0);
		(*tb_filled).insert(row + i, col + 1) = (*t_fill)(i, 1);
		(*tb_filled).insert(row + i, col + 2) = (*t_fill)(i, 2);
		(*tb_filled).insert(row + i, col + 3) = (*t_fill)(i, 3);
	}
}

// helper function to replace values in a sparse matrix with a given 3by3 denses matrix
void RBH::sparse_replace4(Eigen::SparseMatrix<double>* tb_replaced, Eigen::Matrix4d* t_replace, int row, int col) {
	for (int i = 0; i < (*t_replace).rows(); i++) {
		(*tb_replaced).coeffRef(row + i, col) = (*t_replace)(i, 0);
		(*tb_replaced).coeffRef(row + i, col + 1) = (*t_replace)(i, 1);
		(*tb_replaced).coeffRef(row + i, col + 2) = (*t_replace)(i, 2);
		(*tb_replaced).coeffRef(row + i, col + 3) = (*t_replace)(i, 3);
	}
}

// BASIC
// function to convert a ros msg to a sparse matrix
// void RBH::msg_to_matrix(std_msgs::msg::Float64MultiArray min, Eigen::SparseMatrix<double>* mout) {
// 	int rows = min.layout.dim[0].size;
// 	int cols = min.layout.dim[1].size;
// 	float data = 0.0;
// 	std::vector< Eigen::Triplet<double> > tripletList;
// 	tripletList.reserve(64);
// 	for (int i = 0; i < rows; i++) {
// 		for (int j = 0; j < cols; j++) {
// 			data = min.data[i * cols + j];
// 			if (data != 0) {
// 				tripletList.push_back(Eigen::Triplet<double>(i, j, data));
// 			}
// 		}
// 	}
// 	(*mout).setFromTriplets(tripletList.begin(), tripletList.end());
// }

// function to convert from a sparse matrix to a ros msg
// void RBH::matrix_to_msg(Eigen::SparseMatrix<double>* min, std_msgs::msg::Float64MultiArray* mout) {
// 	int rows = min->rows();
// 	int cols = min->cols();
// 	float data = 0.0;
// 	mout->layout.dim[0].size = min->rows();
// 	mout->layout.dim[1].size = min->cols();
// 	mout->data.resize(min->rows() * min->cols());
// 	for (int i = 0; i < rows; i++) {
// 		for (int j = 0; j < cols; j++) {
// 			data = min->coeff(i, j);
// 			if (data != 0) {
// 				mout->data[i * cols + j] = data;
// 			}
// 		}
// 	}
// }

// ADVANCED
// function to convert a ros msg to a sparse matrix
void RBH::msg_to_matrix(std::array<double,768> min, Eigen::SparseMatrix<double>* mout) {
	int rows = min[0];
	int cols = min[1];
	int size = min[2];
	float data = 0.0;
	int i,j = 0;
	std::vector< Eigen::Triplet<double> > tripletList;
	tripletList.reserve(size-1);
	for (int k = 1; k < size; k++) {
		data = min[k * 3 + 2];
		i = min[k * 3 + 0];
		j = min[k * 3 + 1];
		if (data != 0) {
			tripletList.push_back(Eigen::Triplet<double>(i, j, data));
		}
	}
	(*mout).setFromTriplets(tripletList.begin(), tripletList.end());
}

// // function to convert from a sparse matrix to a ros msg
void RBH::matrix_to_msg(Eigen::SparseMatrix<double> min, std::array<double,768>& mout) {
	//std::array<double,216>* mout;
	//double* mout = (double*)malloc(216 * sizeof(double));
	float data = 0.0;
	int i,j = 0;
	int size = min.nonZeros() + 1;
	auto values = min.valuePtr();
	auto inner = min.innerIndexPtr();
	auto outer = min.outerIndexPtr();
	//int size_needed = (size * 3) + 3;
	//std::cout << "size needed: " << size_needed << std::endl;
	mout[0] = min.rows();
	mout[1] = min.cols();
	mout[2] = size;
	for (size_t k = 1; k < size; k++) {
		data = values[k];
		i = inner[k];
		j = outer[k];
		mout[k * 3 + 0] = i;
		mout[k * 3 + 1] = j;
		mout[k * 3 + 2] = data;
	}
}
