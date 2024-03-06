#include <../include/bmn/bubblemethod.h>

namespace API = Haply::HardwareAPI;
using namespace std::chrono_literals;
typedef std::chrono::high_resolution_clock clocky;

// starts up the bmn class
void BMN::bmnav::initbmn(float fbmn, double k_b, double k_i, double d_i, double v_s, double p_s, double b_r, double c_r, double a_d, double v_l, double f_s, double flimx, double flimy, double flimz, double phin_max, double vmaxchange, double PSchange, double VSchange){
    // loop var init
    raw_positions.setZero();
	velocities.setZero();
    positions.setZero();
	abs_positions.setZero();
	forces.setZero();
	Va.setZero();
    D_D_C = { 0, 0, 1.0 };
    V_B_C = { 0, 0, 1.0 };
    A_D_C = { 0, 0, 1.0 };
	phins.setZero();
	dot_phins.setZero();
	rforces.setZero();
	iforces.setZero();
	iforce_list.setZero();

    // loop param init
    count = 0;
    i = 1;
    magVa = 0.0;
    freq = 0.0;

    stiff_k = k_i;
    stiff_ki = k_i;
    damp_b = d_i;
    damp_bi = d_i;
    stiff_kb = k_b;
    bub_rad = b_r;
    con_rad = c_r;
    act_dis = a_d;
    act_dis_in = a_d;
    maxVa = v_l;
    maxVai = v_l;
    phinmax = phin_max;
    vachange = vmaxchange;
    posspringadj = PSchange;
    velspringadj = VSchange;
    flims = flims;
    fscale = f_s;
    p_scale = p_s;
	v_scale = v_s;

    run = true;
    boundary = true;
    ncon = false;
    con = false;
    precon = false;
    postcon = false;

    // inverse startup
    comm = BMN::bmnav::inverse3_setup();
    API::IO::SerialStream stream{ comm.c_str() };
    Inverse_object = API::Devices::Inverse3{&stream};
    h = 1/fbmn;
    w_c = { -0.02, -0.15, 0.1 };
    rest = 0.025;
    BMN::bmnav::centerDevice()
}

// fetches the port the inverse is connected too
std::string BMN::bmnav::inverse3_setup() {
    // find the port the inverse is connected too
    std::string portreturn;
    auto list = API::Devices::DeviceDetection::DetectInverse3s();
    for (const auto& port : list) {
        std::fprintf(stdout, "inverse3: %s\n", port.c_str());
        portreturn = port.c_str();
    }
    if (list.empty()) {
        fprintf(stderr, "no inverse3 detected\n");
    }
    // return it
    return portreturn;
}

// helper function that runs at the beginning to guide the user to the center of the workspace region before starting the simulation
void BMN::bmnav::centerDevice() {
    // wake up the inverse and fetch info
    auto info = Inverse_object.DeviceWakeup();
    std::fprintf(stdout,
        "info: id=%u, model=%u, version={hardware:%u, firmware:%u}\n",
        info.device_id, info.device_model_number,
        info.hardware_version, info.firmware_version);

    // var initializations and declarations
    auto start_time = clocky::now();
    int count = 0;
    double rad = 0.01;
    double stiffy = 30;
    Eigen::Vector3d raw_positions;
    raw_positions.setZero();
    Eigen::Vector3d forces;
    forces.setZero();
    // checks if the end effector is within the deadzone of the center of the workspace
    bool flag = start_check(w_c, &raw_positions, rest);
    Eigen::Vector3d velocities;
    Eigen::Vector3d positions;
    API::Devices::Inverse3::EndEffectorStateResponse state;
    API::Devices::Inverse3::EndEffectorForceRequest requested;
    // loops until the end effector is within the deadzone of the center of the workspace
    while (flag) {
        // calculates the force to apply to the end effector to guide it to the center of the workspace
        count += 1;
        requested.force[0] = forces[0];
        requested.force[1] = forces[1];
        requested.force[2] = forces[2];
        state = Inverse_object.EndEffectorForce(requested);
        raw_positions = { state.position[0], state.position[1], state.position[2] };
        velocities = { state.velocity[0], state.velocity[1], state.velocity[2] };
        force_restitution(w_c, &raw_positions, &rad, &stiffy, &forces);
        flag = start_check(w_c, &raw_positions, rest);

        // frequency limiter
        std::chrono::duration<double> dt = clocky::now() - start_time;
        double freq = 1 / dt.count();
        while (freq > 2000)
        {
			dt = clocky::now() - start_time;
			freq = 1 / dt.count();
		}
        start_time = clocky::now();

        //// outputs the current state to the user
        if (count % 1000 == 0) {
            std::cout << "Not yet within position control region" << std::endl;
        }
    }
}

// takes a step in the simulation updating the position of the drone and the velocity ball then returns the sims current state
void BMN::bmnav::BMNstep() {
    if ((phins.norm() > phinmax)) {
        maxVa = vachange * maxVai;
    } else {
        maxVa = maxVai;
    }

    // inverse 3 interfacing accounting
    requested.force[0] = forces[0];
    requested.force[1] = forces[1];
    requested.force[2] = forces[2];
    state = Inverse_object.EndEffectorForce(requested);
    raw_positions = { state.position[0], state.position[1], state.position[2] };
    velocities = { state.velocity[0], state.velocity[1], state.velocity[2] };

    positions = p_scale * (raw_positions - (w_c));
    abs_positions = positions + V_B_C;

    // Force calculations
    BMN::bmnav::force_restitution(w_c, &raw_positions, &bub_rad, &stiff_bk, &rforces);
    BMN::bmnav::force_interface(&stiff_k, &damp_b, &con_rad, &phins, &dot_phins, &iforces, &iforce_list);
    forces = rforces + iforces;
    BMN::bmnav::force_scaling(&forces, &flims, &fscale);
    BMN::bmnav::force_scaling(&iforce_list, &flims, &fscale);

    // bubble method region checking and handling
    boundary = BMN::bmnav::pos_check(w_c, &raw_positions, &bub_rad);
    double temp = h * v_scale;
    if (boundary == false) {
        BMN::bmnav::velocity_applied(w_c, &raw_positions, &bub_rad, 100, &Va);
        magVa = Va.norm();
        if (magVa > maxVa) {
            Va = { (maxVa / magVa) * Va[0], (maxVa / magVa) * Va[1] , (maxVa / magVa) * Va[2] };
            BMN::bmnav::RK4_vec_update(&V_B_C, Va, temp);
        }
        else {
            BMN::bmnav::RK4_vec_update(&V_B_C, Va, temp);
        }
    }

    // adjust the stiffness and damping of the interface based on the current location of the bubble
    // if the full simulation says the drone is near the wall
    // then the activation distance is set to the bubble radius
    if (precon || con || postcon) {
        //V_B_C = A_D_C - positions;
        stiff_k = stiff_ki;
        damp_b = damp_bi;
        act_dis = 0.0;
    }
    // if the bubble is in the postion control region then
    // the activation distance is set to standard
    else if (boundary && ncon) {
        stiff_k = posspringadj * stiff_ki
        damp_b = posspringadj * damp_bi;
        act_dis = bub_rad;
    }
    // if the bubble is in the velocity control region and we are not near a wall in the full simulation
    // then the activation distance is set
    else if (ncon) {
        stiff_k = velspringadj * stiff_ki;
        damp_b = velspringadj * damp_bi;
        act_dis = act_dis_in;
    }

    // calculates the change in drone and velocity ball position
    // adds the position of the virtual velocity ball to the relative posiiton of the drone
    D_D_C = V_B_C + positions;

    commsmsgs::msg::bmnpub msg{};
    msg.interface_force_list.x = iforce_list[0];
    msg.interface_force_list.y = iforce_list[1];
    msg.interface_force_list.z = iforce_list[2];
    msg.desired_drone_position.x = D_D_C[0];
    msg.desired_drone_position.y = D_D_C[1];
    msg.desired_drone_position.z = D_D_C[2];
    msg.bmn_freq = freq;
    bmn_publisher_->publish(msg);
    
    // frequency limiter
	std::chrono::duration<double> dt = clocky::now() - start_time;
	freq = 1 / dt.count();
    while (freq > (1/h))
    {
        dt = clocky::now() - start_time;
        freq = 1 / dt.count();
    }
    start_time = clocky::now();

    // if (count % 100 == 0) {
    //     // std::cout << "phins: " << phins[0] << ", " << phins[1] << ", " << phins[2] << std::endl;
    //     // std::cout << "dot phins: " << dot_phins[0] << ", " << dot_phins[1] << ", " << dot_phins[2] << std::endl;
    //     std::cout << "interface force: " << iforces[0] << ", " << iforces[1] << ", " << iforces[2] << std::endl;
    //     std::cout << "bubble force: " << rforces[0] << ", " << rforces[1] << ", " << rforces[2] << std::endl;
    //     std::cout << "total force: " << forces[0] << ", " << forces[1] << ", " << forces[2] << std::endl;
    // }

    //count += 1;
    //i += 1;
}

// brim callback processing
void BMN::bmnav::brim_callback(const commsmsgs::msg::brimpub::SharedPtr & msg) {
    phins = { msg->phin_list->x, msg->phin_list->y, msg->phin_list->z };
    dot_phins = { msg->phin_dot_list->x, msg->phin_dot_list->y, msg->phin_dot_list->z };
}

// rbquadsim callback processing
void BMN::bmnav::rbquadsim_callback(const commsmsgs::msg::rbquadsimpub::SharedPtr & msg) {
    con = msg->contact;
    precon = msg->precontact;
    postcon = msg->postcontact;
    ncon = msg->nocontact;
    A_D_C = { msg->position->x, msg->position->y, msg->position->z };
}

// function to calculate the force of contact using RIM
void BMN::bmnav::force_interface(double* stiff, double* damp, double* act_dis, Eigen::Matrix<double, 3, 1>* phins, Eigen::Matrix<double, 3, 1>* dot_phins, Eigen::Vector3d* force, Eigen::Vector3d* force_list) {
    double diff = 0;
    double raw_force = 0;
    // checks if the distance exceeds the activation distance and if so calculates the force
    if (abs((*phins)[0]) > (*act_dis)) {
        //std::cout << "x impact" << std::endl;
        // compression distance for spring force calc
        diff = abs((*phins)[0]) - (*act_dis);
        // find force using a spring damper model
        // !!! spring and damper force may not always act in same direction !!!
        raw_force = (*stiff) * diff + ((*damp) * abs((*dot_phins)[0]));
        (*force_list)[0] = raw_force * copysignf(1, (*phins)[0]);
    }
    else if (abs((*phins)[1]) > (*act_dis)) {
        //std::cout << "y impact" << std::endl;
        diff = abs((*phins)[1]) - (*act_dis);
        raw_force = (*stiff) * diff + ((*damp) * abs((*dot_phins)[1]));
        (*force_list)[1] = raw_force * copysignf(1, (*phins)[1]);
    }
    else if (abs((*phins)[2]) > (*act_dis)) {
        //std::cout << "z impact" << std::endl;
        diff = abs((*phins)[2]) - (*act_dis);
        raw_force = (*stiff) * diff + ((*damp) * abs((*dot_phins)[2]));
        (*force_list)[2] = raw_force * copysignf(1, (*phins)[2]);
    }
    else {
        (*force_list).setZero();
    }
    (*force) = (*force_list);
}

// function to calculate the force of restitution felt by the user when in the velocity control region of the bubble naviation
void BMN::bmnav::force_restitution(Eigen::Vector3d* center, Eigen::Vector3d* device_pos, double* radius, double* stiffness, Eigen::Vector3d* force) {
    Eigen::Vector3d direction;
    Eigen::Vector3d diff;
    // checks if the end effector is within the velocity control region and if so calculates the force
    diff = ((*device_pos) - (*center));
    double distance = diff.norm();
    if (distance < (*radius)) {
        (*force).setZero();
    }
    else {
        // calculates the force using a spring model
        direction = diff / distance;
        (*force) = direction * ((distance - (*radius)) * -(*stiffness));
    }
}

// function to scale/filter forces sent to inverse 3
void BMN::bmnav::force_scaling(Eigen::Vector3d* Force, Eigen::Vector3d* lims, double* scale) {
    (*Force) *= (*scale);
    // if any force is greater than its limit than set it to the limit
    if (abs((*Force)[0]) > (*lims)[0]) {
        (*Force)[0] = (*lims)[0] * copysignf(1, (*Force)[0]);
    }
    if (abs((*Force)[1]) > (*lims)[1]) {
        (*Force)[1] = (*lims)[1] * copysignf(1, (*Force)[1]);
    }
    if (abs((*Force)[2]) > (*lims)[2]) {
        (*Force)[2] = (*lims)[2] * copysignf(1, (*Force)[2]);
    }
}

// helper function to calculate the proper velocity to apply to the velocity ball given the postion of the end effector compared to the surface of
// the postion control region
void BMN::bmnav::velocity_applied(Eigen::Vector3d* center, Eigen::Vector3d* device_pos, double* radius, double scaling, Eigen::Vector3d* Va) {
    Eigen::Vector3d direction;
    Eigen::Vector3d diff;
    // find the direction of the velocity
    diff = ((*device_pos) - (*center));
    double distance = diff.norm();
    direction = diff / distance;
    double abdiff = distance - (*radius);
    // use a cubic function of the distance to calculate the velocity to apply
    (*Va) = direction * (abdiff * abdiff * abdiff * scaling);
}

// helper function to calculate whether the end effector is close enough to the workspace center before starting the simulation
bool BMN::bmnav::start_check(Eigen::Vector3d* center, Eigen::Vector3d* device_pos, double* radiusdead) {
    // find distance between the end effector and the workspace center
    Eigen::Vector3d diff;
    diff = ((*device_pos) - (*center));
    double distance = diff.norm();
    // check if the distance is greater than the deadzone radius and return the result
    if (distance > (*radiusdead)) {
        return true;
    }
    else {
        return false;
    }
}

// helper function to check if the end effector is within the postion control region of the bubble navigation
bool BMN::bmnav::pos_check(Eigen::Vector3d* center, Eigen::Vector3d* device_pos, double* bound_rad) {
    // find distance between the end effector and the workspace center
    Eigen::Vector3d diff;
    diff = ((*device_pos) - (*center));
    double distance = diff.norm();
    // check if the distance is less than the postion control radius and return the result
    if (distance <= (*bound_rad)) {
        return true;
    }
    else {
        return false;
    }
}

// helper function for RK4_vec_update
void BMN::bmnav::RK4_update(double* xn, double* xn_dot, double* h) {
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
void BMN::bmnav::RK4_vec_update(Eigen::Vector3d* xn, Eigen::Vector3d xn_dot, double h) {
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