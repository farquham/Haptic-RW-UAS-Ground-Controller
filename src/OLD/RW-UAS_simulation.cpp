#include "RW-UAS_sim_params.h"
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
// helper funciton to fill a sparse matrix with a 3x3 matrix
Eigen::SparseMatrix<double> sparseFill(Eigen::SparseMatrix<double> tb_filled, Eigen::Matrix3d t_fill, int row, int col) {
    for (int i = 0; i < t_fill.rows(); i++) {
        tb_filled.insert(row + i, col) = t_fill(i, 0);
        tb_filled.insert(row + i, col + 1) = t_fill(i, 1);
        tb_filled.insert(row + i, col + 2) = t_fill(i, 2);
    }
    return tb_filled;
}

// helper function to reset the data transfer struct
void resetVARS(BMN::Var_Transfer* ptr, std::mutex* lock) {
    // reset the data pipe struct
    lock->lock();
    //helper bools
	ptr->mats_ready = false;
	ptr->running = true;
	ptr->contact_check = false;
    ptr->pre_contact_check = false;
    ptr->post_contact_check = false;
    ptr->no_contact_check = false;
	// data logging variables
	ptr->fsim_real = 0.0;
	ptr->fbrim_real = 0.0;
	ptr->fbmn_real = 0.0;
	ptr->fprim_real = 0.0;
	ptr->fpx4_real = 0.0;
	ptr->brim_time = Eigen::Vector2d::Zero();
	ptr->prim_time = Eigen::Vector2d::Zero();
	//RBsystem stuff
    ptr->Drag = Eigen::Vector3d::Zero();
    ptr->Gravity = Eigen::Vector3d::Zero();
    ptr->Interaction = Eigen::Vector3d::Zero();
    ptr->d_pos = {0, 0, 1.0};
    ptr->d_apos = Eigen::Vector3d::Zero();
    ptr->d_vel = Eigen::Vector3d::Zero();
    ptr->d_acc = Eigen::Vector3d::Zero();
    ptr->d_RM = Eigen::Matrix3d::Identity();
    ptr->DDC = {0, 0, 1.0};
	//BMN stuff
    ptr->bint_force_list = Eigen::VectorXd::Zero(3);
    ptr->B_DDC = {0, 0, 1.0};
    ptr->brim_phin_list = Eigen::VectorXd::Zero(3);
    ptr->brim_dot_phin_list = Eigen::VectorXd::Zero(3);
    // PX4 stuff
	ptr->P_DDC = Eigen::Vector3d::Zero();
    ptr->P_DV = Eigen::Vector3d::Zero();
    ptr->P_DA = Eigen::Vector3d::Zero();
	ptr->PPDP = {0, 0, 1.0};
    // lim params
    ptr->BRIM_lims = lims;
    ptr->BRIM_dlims = dlims;
    // sparse matrices
    Eigen::SparseMatrix <double> M_temp = Eigen::SparseMatrix <double>(42, 42);
    for (int i = 0; i < 42 / 6; i++) {
        M_temp = sparseFill(M_temp, Eigen::Matrix3d::Zero(), 3 * i, 3 * i);
    }
    ptr->M_inv = M_temp;
    Eigen::SparseMatrix <double> Ac_temp = Eigen::SparseMatrix <double>(6, 42);
    for (int i = 0; i < 42; i++) {
        Ac_temp.insert(0, i) = 0;
        Ac_temp.insert(1, i) = 0;
        Ac_temp.insert(2, i) = 0;
        Ac_temp.insert(3, i) = 0;
        Ac_temp.insert(4, i) = 0;
        Ac_temp.insert(5, i) = 0;
    }
    ptr->Ac = Ac_temp;
    Eigen::SparseMatrix <double> vb_temp = Eigen::SparseMatrix <double>(1, 42);
    for (int i = 0; i < 42; i++) {
        vb_temp.insert(0, i) = 0;
    }
    ptr->vb = vb_temp;
    lock->unlock();
}

// logs the data from the experiment
void run_logs(BMN::Var_Transfer* ptr, std::mutex* lock) {
    // pausing loop until the full similation is ready
    while ((brim_ready == false) || (prim_ready == false) || (vis_ready == false) || (wait == true)){
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    Eigen::Vector3d SDVSP = Eigen::Vector3d::Zero();
    Eigen::Vector3d SDVER = Eigen::Vector3d::Zero();
    Eigen::Vector3d SDVINT = Eigen::Vector3d::Zero();
    Eigen::Vector3d SDVDOT = Eigen::Vector3d::Zero();
    Eigen::Vector3d SDASP = Eigen::Vector3d::Zero();

    Eigen::Vector3d SDAP = Eigen::Vector3d::Zero();

    // no, pre, con, post
    bool nc = true;
    bool prc = false;
    bool cc = false;
    bool poc = false;

    // setup the log files for acctual drone
    // ADP_log_file.open(ADP_file, std::ios::out | std::ios::app);
    ADV_log_file.open(ADV_file, std::ios::out | std::ios::app);
    ADA_log_file.open(ADA_file, std::ios::out | std::ios::app);
    // setup the log files for simulated drone
    SDP_log_file.open(SDP_file, std::ios::out | std::ios::app);
    SDV_log_file.open(SDV_file, std::ios::out | std::ios::app);
    SDA_log_file.open(SDA_file, std::ios::out | std::ios::app);
    // setup the log files for bubble method RIM
    BDDP_log_file.open(BDDP_file, std::ios::out | std::ios::app);
    //BPDP_log_file.open(BPDP_file, std::ios::out | std::ios::app);
    INTFOR_log_file.open(INTFOR_file, std::ios::out | std::ios::app);
    // setup the log files for frequency data
    FREQ_log_file.open(FREQ_file, std::ios::out | std::ios::app);
    BRtime_log_file.open(BRtime_file, std::ios::out | std::ios::app);
    //PRtime_log_file.open(PRtime_file, std::ios::out | std::ios::app);
    contact_log_file.open(contact_file, std::ios::out | std::ios::app);

    // some loop vars
    auto start_time = clocky::now();
    double time = 0.0;
    double freq = 0;
    // while loop that keeps running until the stop button is pressed
    std::cout << "Starting logging..." << std::endl;
    while (run) {
        // import the data from the other threads
        lock->lock();
        // drone stuff
        // ADP = ptr->P_DDC;
        // ADV = ptr->P_DV;
        // ADA = ptr->P_DA;
        SDP = ptr->d_pos;
        SDV = ptr->d_vel;
        SDA = ptr->d_acc;
        // rim stuff
        BDDP = ptr->B_DDC;
        //BPDP = ptr->PPDP; !!
        IFOR[0] = ptr->bint_force_list[0];
        IFOR[1] = ptr->bint_force_list[1];
        IFOR[2] = ptr->bint_force_list[2];
        // Frequency stuff
        FREQ[0] = ptr->fsim_real;
        FREQ[1] = ptr->fbrim_real;
        FREQ[2] = ptr->fbmn_real;
        FREQ[3] = ptr->fprim_real;
        FREQ[4] = ptr->fpx4_real;
        FREQ[5] = freq;
        BRcount = ptr->brim_time[0];
        BRtime = ptr->brim_time[1];
        // PRcount = ptr->prim_time[0];
        // PRtime = ptr->prim_time[1];
        // controller debugging
        SDVSP = ptr->vel_sp;
        SDVER = ptr->vel_err;
        SDVINT = ptr->vel_int;
        SDVDOT = ptr->vel_dot;
        SDASP = ptr->acc_sp;
        SDAP = ptr->d_apos;
        nc = ptr->no_contact_check;
        prc = ptr->pre_contact_check;
        cc = ptr->contact_check;
        poc = ptr->post_contact_check;
        lock->unlock();

        time = time + 1 / fcom2;

        // log the px4 data
        // ADP_log_file << ADP[0] << "," << ADP[1] << "," << ADP[2] << "," << time << "\n";
        // ADV_log_file << ADV[0] << "," << ADV[1] << "," << ADV[2] << "," << time << "\n";
        // ADA_log_file << ADA[0] << "," << ADA[1] << "," << ADA[2] << "," << time << "\n";
        // all the vel controller info
        ADV_log_file << SDVSP[0] << "," << SDVSP[1] << "," << SDVSP[2] << "," << SDVER[0] << "," << SDVER[1] << "," << SDVER[2] << "," << SDVINT[0] << "," << SDVINT[1] << "," << SDVINT[2] << "," << SDVDOT[0] << "," << SDVDOT[1] << "," << SDVDOT[2] << "," << time << "\n";
        // the acc vel controller info
        ADA_log_file << SDASP[0] << "," << SDASP[1] << "," << SDASP[2] << "," << time << "\n";
        // log the sim data
        SDP_log_file << SDP[0] << "," << SDP[1] << "," << SDP[2] << "," << time << "," << SDAP[0] << "," << SDAP[1] << "," << SDAP[2] << "\n";
        SDV_log_file << SDV[0] << "," << SDV[1] << "," << SDV[2] << "," << time << "\n";
        SDA_log_file << SDA[0] << "," << SDA[1] << "," << SDA[2] << "," << time << "\n";
        // log the rim data
        BDDP_log_file << BDDP[0] << "," << BDDP[1] << "," << BDDP[2] << "," << time << "\n";
        //BPDP_log_file << BPDP[0] << "," << BPDP[1] << "," << BPDP[2] << "," << time << "\n";
        INTFOR_log_file << IFOR[0] << "," << IFOR[1] << "," << IFOR[2] << "," << time << "\n";
        // log frequency data
        FREQ_log_file << FREQ[0] << "," << FREQ[1] << "," << FREQ[2] << "," << FREQ[3] << "," << FREQ[4] << "," << FREQ[5] << "," << time << "\n";
        BRtime_log_file << BRcount << "," << BRtime << "\n";
        //PRtime_log_file << PRcount << "," << PRtime << "\n";
        contact_log_file << nc << "," << prc << "," << cc << "," << poc << "," << time << "\n";

        // loop frequency tracking and limiting
        std::chrono::duration<double> dt = clocky::now() - start_time;
        freq = 1 / dt.count();
        while (freq > fcom2)
        {
            dt = clocky::now() - start_time;
            freq = 1 / dt.count();
        }
        start_time = clocky::now();

        // uncomment to output info about the loop
        // if (time_step % 100 == 0) {
        //     std::cout << "simulated position: " << Drone_pos[0] << ", " << Drone_pos[1] << ", " << Drone_pos[2] << std::endl;
        //     //std::cout << "actual position: " << ADP[0] << ", " << ADP[1] << ", " << ADP[2] << std::endl;
        // }

        time_step += 1;
    }
}

// function which handles the communication between the different threads primarily the visualization and the starting and stopping
void run_comms(BMN::Var_Transfer* ptr, std::mutex* lock) {
    while (wait) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // some loop vars needed
    int count = 0;
    auto start_time = clocky::now();
    std::cout << "Starting communication..." << std::endl;
    while (run_comms) {

        // import data from other threads
        lock->lock();
        // drone data needed for visualization loop
        Drone_pos = ptr->d_pos;
        Drone_RM = ptr->d_RM;
        // boolean used to stop the program and save the data
        ptr->running = run_check;
        run = ptr->running;
        lock->unlock();

        // reset all the values in the data pipe struct
        if (reset_data_pipe) {
            // call the reset function for the data struct
            resetVARS(ptr, lock);
            reset_data_pipe = false;
        }

        // loop frequency tracking and limiting
        std::chrono::duration<double> dt = clocky::now() - start_time;
        double freq = 1 / dt.count();
        while (freq > fvis)
        {
            dt = clocky::now() - start_time;
            freq = 1 / dt.count();
        }
        start_time = clocky::now();

        // uncomment to output loop info
        /*if (count % 1000 == 0) {
            std::cout << "COMM Frequency: " << freq << std::endl;
        }*/
        count += 1;

        if (run == false) {
            run_comm_loop = false;
        }
    }
}

// function to start and run the bubble method navigation loop
void run_bmn(BMN::Var_Transfer* ptr, std::mutex* lock) {
    while ((vis_ready == false)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // Inverse 3 setup
    std::string comm = BMN::inverse3_setup();

    API::IO::SerialStream stream{ comm.c_str() };
    API::Devices::Inverse3 Inverse_object{&stream};
    
    // Initial Values
    double h = 1 / fbmn;
    Eigen::Vector3d w_c = { -0.02, -0.15, 0.1 };
    BMN::setup_parameters paras;
    paras.k_b = bubble_stiffness;
    paras.k_i = interface_stiffness;
    paras.d_i = interface_damping;
    paras.p_s = position_scaling;
    paras.v_s = velocity_scaling;
    paras.b_r = bubble_radius;
    paras.s_s = h;
    paras.f_com = fcom2;
    paras.c_r = contact_radius;
    paras.a_d = activation_distance;
    paras.v_l = PIDlims[6]*0.1;

    paras.f_s = f_scale;
    paras.flims = flimits;
    paras.phin_max = phn_max;
    paras.vmaxchange = vmaxch;
    paras.PSchange = PSch;
    paras.VSchange = VSch;

    double temp = 0.025;

    // center the inverse before using to ensure ideal performance
    BMN::centerDevice(Inverse_object, &w_c, &temp);

    // // static desired position for testing
    // Eigen::Vector3d DDC = {0.0, 0.0, 2.0};
    // lock->lock();
    // ptr->B_DDC = DDC;
    // ptr->DDC = DDC;
    // lock->unlock();
    bmn_ready = true;
    std::cout << "bmn_ready: " << bmn_ready << std::endl;
    // while (run) {
    //     lock->lock();
    //     ptr->B_DDC = DDC;
    //     lock->unlock();
    // }

    // run the bubble method loop
    BMN::BMNLoop(Inverse_object, &w_c, ptr, lock, &paras);
}

// function to start and run the BRIM loop
void run_BRIM(BMN::Var_Transfer* ptr, std::mutex* lock) {
    while (wait) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // setup the BRIM system and check if the required sparse matrices have been initialized
    BRIM::BRIM brimsys = BRIM::BRIM(fbrim);
    bool check = false;
    lock->lock();
    check = ptr->mats_ready;
    lock->unlock();

    // pausing loop until the full similation is ready and the related matrices are as well
    while ((sim_ready == false) && (check == false)){
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        lock->lock();
        check = ptr->mats_ready;
        lock->unlock();
    }

    // outputs that the BRIM system is ready
    brim_ready = true;
    std::cout << "brim_ready: " << brim_ready << std::endl;

    // starts the BRIM loop
    brimsys.BRIMLoop(ptr, lock, fcom, fcom2, rim_type);
}

// function to start and run the PX4 communications loop
void run_px4(BMN::Var_Transfer* ptr, std::mutex* lock) {
    while (wait) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // vars needed to get init to work
    int arc = 0;
    // start the rclcpp ROS2 control
    std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(arc, nullptr);

    // indicate that the PX4 system is ready
    px4_ready = true;
    std::cout << "px4_ready: " << px4_ready << std::endl;

    // startup the ROS2 node and give it data pipe struct pointer
	rclcpp::spin(std::make_shared<PX4C::OffboardControl>(ptr, lock));

    // stop the node
	rclcpp::shutdown();
}

// function to start and run the PRIM loop
void run_PRIM(BMN::Var_Transfer* ptr, std::mutex* lock) {
    while (wait) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // Before control of the IRL drone can be given, it must be taken off and put into position at 1m z
    // vars needed for the IRL drone prep
    Eigen::Vector3d IRL_pos = {0, 0, 0};
    Eigen::Vector3d IRL_goal = {0, 0, 1.0};
    double diffnorm = 10;
    prim_ready = true;
    std::cout << "prim_ready: " << prim_ready << std::endl;
    // IRL drone prep, make it take off to 1m z and wait
    while ((px4_ready) && (!prim_ready)) {
        // to check if the drone is at the goal position
        diffnorm = (IRL_pos - IRL_goal).norm();

        // to stop the loop if the drone is at the goal position
        if (diffnorm < 0.01) {
            irl_drone_ready = true;
        }

        // if the drone is in the correct pos out that and stop this loop
        if (irl_drone_ready) {
            lock->lock();
            ptr->PPDP = {0, 0, 1.0};
            lock->unlock();
            prim_ready = true;
        }
        // otherwise keep moving the drone
        else {
            lock->lock();
            IRL_pos = ptr->P_DDC;
            lock->unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    // initialize the PRIM and check if the required sparse matrices have been initialized
    PRIM::PRIM primsys = PRIM::PRIM(fprim);
    bool check = false;
    lock->lock();
    check = ptr->mats_ready;
    lock->unlock();

    // pausing loop until the full similation is ready and the related matrices are as well
    while ((sim_ready == false) || (check == false)){
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        lock->lock();
        check = ptr->mats_ready;
        lock->unlock();
    }

    // start the PRIM loop
    primsys.PRIMLoop(ptr, lock, fcom3, fcom, rim_type);
}

// function to start and run the main dynamics simulation loop
void run_sim(BMN::Var_Transfer* ptr, std::mutex* lock) {
    while (wait) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
	// run simulation
    //contact plane equations setup (6 planes) (pos x, neg x, pos y, neg y, pos z, neg z)
    double poswalldis = 5.0;
    double negwalldis = -5.0;
    Eigen::Matrix<double, 6, 3> n_vecs;
    n_vecs << -1, 0, 0,
        1, 0, 0,
        0, -1, 0,
        0, 1, 0,
        0, 0, -1,
        0, 0, 1;
    Eigen::Matrix<double, 6, 1> ds;
    ds << (-1 * poswalldis),
        (1 * negwalldis),
        (-1 * poswalldis),
        (1 * negwalldis),
        (-2 * poswalldis),
        0; // moving the plane to be at the origin
    Eigen::Matrix<double, 6, 3> n_hats;
    n_hats << -1, 0, 0,
        1, 0, 0,
        0, -1, 0,
        0, 1, 0,
        0, 0, -1,
        0, 0, 1;
    RBH::planes plane_equations;
    plane_equations.n = n_vecs;
    plane_equations.d = ds;
    plane_equations.nhat = n_hats;
    // initialize drone object
    Quadcopter::dr_prop dr_prop;
    
    // starting position is after takeoff
    dr_prop.g_p = { 0,0, 1.0 };

    // drone properties copied from global vars to drone struct
    dr_prop.l = length;
    dr_prop.w = width;
    dr_prop.h = height;
    dr_prop.mass = drone_mass;
    dr_prop.con_rad = contact_radius;
    dr_prop.prop_dia = prop_diameter;
    //double d_yaw = 0.0;
    dr_prop.k = k_prop;
    dr_prop.b = b_prop;
    dr_prop.xypdis = xy_p_dis;
    dr_prop.zpdis = z_p_dis;
    dr_prop.Tscale = T_scale;
    dr_prop.PIDlims = PIDlims;
    // In
    dr_prop.In(0, 0) = Inr1[0];
    dr_prop.In(0, 1) = Inr1[1];
    dr_prop.In(0, 2) = Inr1[2];
    dr_prop.In(1, 0) = Inr2[0];
    dr_prop.In(1, 1) = Inr2[1];
    dr_prop.In(1, 2) = Inr2[2];
    dr_prop.In(2, 0) = Inr3[0];
    dr_prop.In(2, 1) = Inr3[1];
    dr_prop.In(2, 2) = Inr3[2];
    // done In
    dr_prop.kp = kp_pos;
    dr_prop.kvp = kp_vel;
    dr_prop.kvi = ki_vel;
    dr_prop.kvd = kd_vel;
    dr_prop.kap = kp_att;
    dr_prop.krp = kp_rate;
    dr_prop.kri = ki_rate;
    dr_prop.krd = kd_rate;
    dr_prop.krff = ff_rate;

    dr_prop.vel_con_lims = conlimsvel;

    // pausing loop until all other threads have returned ready status
    while ((vis_ready == false) || (bmn_ready == false) || (px4_ready == false) || (prim_ready == false)) {
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}

    // initialize rigid body system and setup some variables
    double h = 1 / fsim;
    double stiffness = wall_stiffness;
    double damping = wall_damping;
    RBsystem::RBsystem rigidbody = RBsystem::RBsystem(&dr_prop, &plane_equations, h, stiffness, damping);

    // indicate that the simulation is ready
    sim_ready = true;
    std::cout << "sim_ready: " << sim_ready << std::endl;
    
    //run the rigid body simulation loop
    rigidbody.RBLoop(ptr, lock, fcom);
}

// function to start the simulation
void startSimulation()
{
    // extract the ID number and test number from the input boxes and the rim type and test direction from the combo boxes
    string r_type = std::to_string(rim_type);
    string exp_num = Test_num;
    string Part_ID = ID_num;
    string type = "FM";
    if (Test_type == 1) {
        type = "GM";
    }

    // setup the log file names with the four inputs provided
    ADP_file = "/home/farquham/Desktop/RW-sim/Experimental_Data/Multi User Study/" + Part_ID + "/EXP_" + exp_num + "/log_E_" + exp_num + "_M_" + type + "_R_" + r_type + "_1_ADP.csv";
    ADV_file = "/home/farquham/Desktop/RW-sim/Experimental_Data/Multi User Study/" + Part_ID + "/EXP_" + exp_num + "/log_E_" + exp_num + "_M_" + type + "_R_" + r_type + "_2_ADV.csv";
    ADA_file = "/home/farquham/Desktop/RW-sim/Experimental_Data/Multi User Study/" + Part_ID + "/EXP_" + exp_num + "/log_E_" + exp_num + "_M_" + type + "_R_" + r_type + "_3_ADA.csv";
    SDP_file = "/home/farquham/Desktop/RW-sim/Experimental_Data/Multi User Study/" + Part_ID + "/EXP_" + exp_num + "/log_E_" + exp_num + "_M_" + type + "_R_" + r_type + "_4_SDP.csv";
    SDV_file = "/home/farquham/Desktop/RW-sim/Experimental_Data/Multi User Study/" + Part_ID + "/EXP_" + exp_num + "/log_E_" + exp_num + "_M_" + type + "_R_" + r_type + "_5_SDV.csv";
    SDA_file = "/home/farquham/Desktop/RW-sim/Experimental_Data/Multi User Study/" + Part_ID + "/EXP_" + exp_num + "/log_E_" + exp_num + "_M_" + type + "_R_" + r_type + "_6_SDA.csv";
    BDDP_file = "/home/farquham/Desktop/RW-sim/Experimental_Data/Multi User Study/" + Part_ID + "/EXP_" + exp_num + "/log_E_" + exp_num + "_M_" + type + "_R_" + r_type + "_7_BDDP.csv";
    BPDP_file = "/home/farquham/Desktop/RW-sim/Experimental_Data/Multi User Study/" + Part_ID + "/EXP_" + exp_num + "/log_E_" + exp_num + "_M_" + type + "_R_" + r_type + "_8_BPDP.csv";
    INTFOR_file = "/home/farquham/Desktop/RW-sim/Experimental_Data/Multi User Study/" + Part_ID + "/EXP_" + exp_num + "/log_E_" + exp_num + "_M_" + type + "_R_" + r_type + "_9_IFOR.csv";
    FREQ_file = "/home/farquham/Desktop/RW-sim/Experimental_Data/Multi User Study/" + Part_ID + "/EXP_" + exp_num + "/log_E_" + exp_num + "_M_" + type + "_R_" + r_type + "_10_FREQ.csv";
    BRtime_file = "/home/farquham/Desktop/RW-sim/Experimental_Data/Multi User Study/" + Part_ID + "/EXP_" + exp_num + "/log_E_" + exp_num + "_M_" + type + "_R_" + r_type + "_11_BRT.csv";
    PRtime_file = "/home/farquham/Desktop/RW-sim/Experimental_Data/Multi User Study/" + Part_ID + "/EXP_" + exp_num + "/log_E_" + exp_num + "_M_" + type + "_R_" + r_type + "_12_PRT.csv";
    contact_file = "/home/farquham/Desktop/RW-sim/Experimental_Data/Multi User Study/" + Part_ID + "/EXP_" + exp_num + "/log_E_" + exp_num + "_M_" + type + "_R_" + r_type + "_13_CON.csv";

    // start the threads
    start_loops = true;
}

// function to stop the simulation
void stopSimulation()
{
    // setting this to false should end all the while loops throughout the program
    // this unfortunately does not work for the PX4 thread as there is no while loop
    run_check = false;
    wait = true;
}

// function to reset the simulation
void resetSimulation()
{
    // stop the running simulation
    stopSimulation();

    // clearing global vars
    Drone_pos = {0, 0, 1.0};
    Drone_pos_pre = {0, 0, 1.0};
    Drone_RM = Eigen::Matrix3d::Identity();
    // logging vars resetting
    ADP.setZero();
    ADV.setZero();
    ADA.setZero();
    SDP.setZero();
    SDV.setZero();
    SDA.setZero();
    BDDP.setZero();
    BPDP.setZero();
    IFOR.setZero();
    FREQ.setZero();
    BRtime = 0.0;
    BRcount = 0.0;
    PRtime = 0.0;
    PRcount = 0.0;

    // clearing out the data pipe struct
    reset_data_pipe = true;

    // resetting the helper bools
    sim_ready = false;
    bmn_ready = false;
    brim_ready = false;
    px4_ready = false;
    prim_ready = false;
    irl_drone_ready = false;

    run = false;
    run_check = false;
    start_loops = false;

    // start a fresh simulation
    //startSimulation();
}

// function to save the log files and close them
void saveLogs()
{
    // to save they just have to be properly closed
    ADP_log_file.close();
    ADV_log_file.close();
    ADA_log_file.close();

    SDP_log_file.close();
    SDV_log_file.close();
    SDA_log_file.close();

    BDDP_log_file.close();
    //BPDP_log_file.close();
    INTFOR_log_file.close();

    FREQ_log_file.close();
    BRtime_log_file.close();
    //PRtime_log_file.close();
    contact_log_file.close();
}

// function to overwrite garbage log files
void overwriteLogs() {
    // first close the files
    ADP_log_file.close();
    ADV_log_file.close();
    ADA_log_file.close();

    SDP_log_file.close();
    SDV_log_file.close();
    SDA_log_file.close();

    BDDP_log_file.close();
    //BPDP_log_file.close();
    INTFOR_log_file.close();

    FREQ_log_file.close();
    BRtime_log_file.close();
    //PRtime_log_file.close();
    contact_log_file.close();

    // then open them in overwrite mode (truncation mode)
    ADP_log_file.open(ADP_file, std::ios::out | std::ios::trunc);
    ADV_log_file.open(ADV_file, std::ios::out | std::ios::trunc);
    ADA_log_file.open(ADA_file, std::ios::out | std::ios::trunc);

    SDP_log_file.open(SDP_file, std::ios::out | std::ios::trunc);
    SDV_log_file.open(SDV_file, std::ios::out | std::ios::trunc);
    SDA_log_file.open(SDA_file, std::ios::out | std::ios::trunc);

    BDDP_log_file.open(BDDP_file, std::ios::out | std::ios::trunc);
    //BPDP_log_file.open(BPDP_file, std::ios::out | std::ios::trunc);
    INTFOR_log_file.open(INTFOR_file, std::ios::out | std::ios::trunc);

    FREQ_log_file.open(FREQ_file, std::ios::out | std::ios::trunc);
    BRtime_log_file.open(BRtime_file, std::ios::out | std::ios::trunc);
    //PRtime_log_file.open(PRtime_file, std::ios::out | std::ios::trunc);
    contact_log_file.open(contact_file, std::ios::out | std::ios::trunc);

    // then close them again
    ADP_log_file.close();
    ADV_log_file.close();
    ADA_log_file.close();

    SDP_log_file.close();
    SDV_log_file.close();
    SDA_log_file.close();

    BDDP_log_file.close();
    //BPDP_log_file.close();
    INTFOR_log_file.close();

    FREQ_log_file.close();
    BRtime_log_file.close();
    //PRtime_log_file.close();
    contact_log_file.close();
}

// function which handles the visualization updating
void callback() {
    ImGui::Begin("Simulation Controls"); // title of gui box
    ImGui::PushItemWidth(100); // Make ui elements 100 pixels wide,
    ImGui::SetWindowFontScale(1.5); // Make ui elements 150% of their normal size
    // ID number input box
    ImGui::InputText("ID Number", ID_num, IM_ARRAYSIZE(ID_num));
    // test number input box
    ImGui::InputText("Test Number", Test_num, IM_ARRAYSIZE(Test_num));
    // Test direction select menu
    ImGui::Combo("Test Type", &Test_type, "Free Movement\0Guided Movement\0");
    // RIM type select menu
    ImGui::Combo("Co-Sim Type", &rim_type, "ZOH\0FOH\0RIM\0");
    // start simulation button using above ID number, and option selections
    if (ImGui::Button("Start Simulation")) {
        if (!run) {
            std::cout << "start Button pressed" << std::endl;
            startSimulation();
        }
    }
    // reset simulation button (all vars)
    if (ImGui::Button("Reset Simulation")) {
        std::cout << "reset Button pressed" << std::endl;
        resetSimulation();
    }
    // save logs button using above ID number and selected options
    if (ImGui::Button("Save Logs")) {
        std::cout << "save Button pressed" << std::endl;
        saveLogs();
    }
    // overwrite logs button using above ID number and selected options (used when something goes wrong and data is bad and not kept)
    if(ImGui::Button("Overwrite Logs")) {
        std::cout << "overwrite Button pressed" << std::endl;
        overwriteLogs();
    }
    // stop simulation button
    if (ImGui::Button("Stop Simulation")) {
        if (run) {
            std::cout << "stop Button pressed" << std::endl;
            stopSimulation();
        }
    }
    ImGui::PopItemWidth();
    ImGui::End();

    // updates the drone position graphically using the global vars that are updated in the comm thread
    const int drows = DroneVOG.rows();
    Eigen::MatrixXd d;
    d = Eigen::MatrixXd::Ones(drows, 1);
    Eigen::MatrixXd DroneVnew(drows, 3);
    DroneVnew << (-Drone_pos[0]) * d, (Drone_pos[2]-5)* d, (Drone_pos[1])* d;
    DroneV = (Drone_RM * DroneVOG.transpose()).transpose() + (1000 * DroneVnew);
    polyscope::getSurfaceMesh("Drone")->polyscope::SurfaceMesh::updateVertexPositions(DroneV);
}

// function which starts up the polyscope visualization
void run_vis() {
	// run visualization
    // Options
    polyscope::options::autocenterStructures = true;
    polyscope::options::buildGui = false;
    polyscope::view::windowWidth = 3500;
    polyscope::view::windowHeight = 2200;
    //polyscope::view::projectionMode = polyscope::ProjectionMode::Orthographic;

    // Initialize polyscope
    polyscope::init();

    // set the position and direction of the camera
    polyscope::view::lookAt(glm::vec3{-2.0, 1.5, -50.0}, glm::vec3{0.0, 0.0, 0.0});

    // Read the mesh
    igl::readOBJ("/home/farquham/Desktop/RW-sim/Hardware_simulation/src/RW-UAS_simulation/Holybro_np.obj", DroneV, DroneF);
    igl::readOBJ("/home/farquham/Desktop/RW-sim/Hardware_simulation/src/RW-UAS_simulation/Room_Large_Zn.obj", RoomV, RoomF);

    // adjust the mesh to work with polyscopes coordinate system
    DroneVOG = DroneV;
    Eigen::Matrix3d rot_swap = Eigen::Matrix3d::Identity();
    rot_swap << 0, 1, 0, 0, 0, 1, 1, 0, 0;
    DroneV = (rot_swap * DroneVOG.transpose()).transpose();
    DroneVOG = DroneV;

    // Register the mesh with Polyscope
    polyscope::registerSurfaceMesh("Room", RoomV, RoomF);
    polyscope::registerSurfaceMesh("Drone", DroneV, DroneF);

    // change the colors to be more visually appealing
    polyscope::getSurfaceMesh("Drone")->setSurfaceColor(glm::vec3{ 0.2, 0.2, 0.2 });
    polyscope::getSurfaceMesh("Room")->setSurfaceColor(glm::vec3{ 0.8, 0.8, 0.8 });

    vis_ready = true;
    std::cout << "vis_ready: " << vis_ready << std::endl;

    // Add the callback
    polyscope::state::userCallback = callback;

    // Show the gui
    polyscope::show();
}

// function to seperate all other threads from the visualization thread
void run_all(BMN::Var_Transfer* ptr, std::mutex* lock) {
    while (true) {
        if (start_loops) {
            std::cout << "pre loop stuff" << std::endl;

            resetVARS(ptr, lock);

            // start threads
            std::thread th0(run_logs, ptr, lock);

            std::thread th1(run_comms, ptr, lock);
	
	        std::thread th2(run_bmn, ptr, lock);

            std::thread th3(run_BRIM, ptr, lock);

            std::thread th5(run_PRIM, ptr, lock);
	
	        std::thread th6(run_sim, ptr, lock);

            // indicate that the simulation is running
            run_check = true;
            run = true;

            std::cout << "loops starting" << std::endl;

            th0.detach();

            th1.detach();
	
	        th2.detach();
	
            th3.detach();

            th5.detach();

            th6.detach();

            std::cout << "loops started" << std::endl;

            start_loops = false;
            wait = false;

            std::cout << "wait: " << wait << std::endl;
        }
    }
}

// main function which the vis program which then starts all the treads using the start button
int main()
{
    // start the visualization thread
    std::thread thA(run_vis);

    thA.detach();

    // setup the data pipe struct
    BMN::Var_Transfer* ptr = &current_state;
    std::mutex lock;

    // setup the data pipe struct with default values
	resetVARS(ptr, &lock);

    std::thread th4(run_px4, ptr, &lock);

    th4.detach();

    first_run = false;

    run_all(ptr, &lock);

	return 0;
}