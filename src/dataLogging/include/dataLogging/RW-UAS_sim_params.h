#include "RW-UAS_simulation.h"

// getting the haply and chrono libraries ready for use
namespace API = Haply::HardwareAPI;
using namespace std::chrono_literals;
typedef std::chrono::high_resolution_clock clocky;
using namespace std;

// parameters to adjust
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

// bmn parameters
double position_scaling = 2.0;          // used in position control mode  PROB FINE
double velocity_scaling = 3.0;          // used in velocity control mode  PROB FINE
double bubble_radius = 0.05;            // size of bubble region used     PROB FINE
// TOO TUNE
double activation_distance = bubble_radius * position_scaling * 2;      // distance from center of bubble to start applying interface force (0.15 - 0.25)
double bubble_stiffness = 30.0;         // stiffness of bubble (10.0 - 50.0)
double interface_stiffness = 100.0;     // stiffness of interface (10.0 - 100.0)
double interface_damping = 10.0;        // damping of interface (1.0 - 10.0)

double f_scale = 1.0;
Eigen::Vector3d flimits = {6,6,6};
double phn_max = 0.4;
double vmaxch = 10.0;
double PSch = 0.2;
double VSch = 0.1;
//v1: PSch = 0.2, VSch = 0.1 vmaxch = 10.0
//v2: PSch = 0.25, VSch = 0.15 vmaxch = 10.0
//v3: PSch = 0.2, VSch = 0.1 vmaxch = 5.0
// TOO TUNE

// BRIM parameters
Eigen::Vector3d lims = { 5.0, 5.0, 5.0};
Eigen::Vector3d dlims = { 12.0, 12.0, 3.0};

// sim parameters
double wall_stiffness = 100.0; // wall stiffness
double wall_damping = 10.0;    // wall damping

// drone parameters
//// Holybro drone properties
// given props
double length = 0.3535; // length of drone from prop center to prop center
double width = 0.3535; // same as length typically
double height = 0.306; // height of drone from bottom feet to top
double drone_mass = 1.5; // mass of the drone in kg (drone, props, batteries, rpi, etc.)
double contact_radius = 0.5; // radius of the drone's contact sphere
double prop_diameter = 0.254; // the diameter of the props used on the drone
//double d_yaw = 0.0;
// calc props
// double k_prop = 1.2843e-6; old vals for simple thrust model
// double b_prop = 1.80137e-8;
double k_prop = (1.2843e-6)/(1.2*(std::pow(prop_diameter,4))); // thrust coefficient
double b_prop = (1.80137e-8)/(1.2*(std::pow(prop_diameter,5)));// torque coefficient
double xy_p_dis = 0.177; // the x/y distance between the prop center and the drone center
double z_p_dis = 0.0933; // the z distance between the prop center and the drone center
// A simplified (only diagonal elements) inertia matrix for the drone
Eigen::Vector3d Inr1 = { 0.02166666666666667, 0.0, 0.0 }; //!!!
Eigen::Vector3d Inr2 = { 0.0, 0.02166666666666667, 0.0 };  //!!!
Eigen::Vector3d Inr3 = { 0.0, 0.0, 0.04000000000000001 };  //!!!!
// tune props
double T_scale = 1; // scaling factor for thrust if needed
// limits for the Controller
// 0.0->1800, 0.0->1800, 0.0->1800, 0.0->1, 0.0->1, 0.0->1, 0->20, 0.5->8, 0.5->4, 0.05->0.5, 0->1, 20->89, 0.0->0.5
// att_limsx,y,z, rate_limsx,y,z, lim_vel_horz, lim_vel_up, lim_vel_down, lim_thr_min, lim_thr_max, lim_tilt, lim_thr_xy_margin
// tot thr = 52.26768, 52.27 = lim_thr_max, 6.27 = lim_thr_min, 15.68 = lim_thr_xy_margin
double attlimxy = 220 * (3.14159/180);
double attlimz = 200 * (3.14159/180);
Eigen::Matrix <double, 13,1> PIDlims= { attlimxy, attlimxy, attlimz, 0.3, 0.3, 0.3, 12.0, 3.0, 1.5, 6.27, 52.27, 45.0, 15.68 };
// recommended values from px4 github
// tuned values using auto tuner
// x: (0.0->2 = 0.95), y: (0.0->2 = 0.95), z: (0.1->1.5 = 1.0)
Eigen::Vector3d kp_pos = { 1.25, 1.25, 4.47 };
// x: (1.2->5 = 1.8), y: (1.2->5 = 1.8), z: (2->15 = 4.0)
Eigen::Vector3d kp_vel = { 2.72, 2.7, 7.56 };
// x: (0.0->60 = 0.4), y: (0.0->60 = 0.4), z: (0.2->3 = 2.0)
Eigen::Vector3d ki_vel = { 1.08, 1.07, 1.74 };
// x: (0.1->2 = 0.2), y: (0.1->2 = 0.2), z: (0.0->2 = 0.0)
Eigen::Vector3d kd_vel = { 0.56, -0.33, 0.45 };
// roll: (0.0->12 = 6.5), pitch: (0.0->12 = 6.5), yaw: (0.0->5 = 2.8)
Eigen::Vector3d kp_att = { 6.0, 6.0, 2.5 };
// roll: (0.01->0.5 = 0.15), pitch: (0.01->0.6 = 0.15), yaw: (0.0->0.6 = 0.2)
Eigen::Vector3d kp_rate = { 0.262, 0.303, 0.309 };
// roll: (0.0->0.5 = 0.2), pitch: (0.0->0.6 = 0.2), yaw: (0.0->0.6 = 0.1)
Eigen::Vector3d ki_rate = { 0.254, 0.305, 0.304 };
// roll: (0.0->0.01 = 0.003), pitch: (0.0->0.01 = 0.003), yaw: (0.0->0.01 = 0.0)
Eigen::Vector3d kd_rate = { -0.015, 0.0026, 0.0006 };
// roll: (0.0->0.5 = 0.0), pitch: (0.0->0.6 = 0.0), yaw: (0.0->0.6 = 0.0)
Eigen::Vector3d ff_rate = { -0.002, -0.01, 0.004 };

// vel int, vel err lims
Eigen::Vector2d conlimsvel = { 1.0, 1.0 };

// DroneVolt drone properties
    //dr_prop.l = 1.25;
    //dr_prop.w = 1.25;
    //dr_prop.h = 0.5;
    //dr_prop.mass = 10;
    //dr_prop.con_rad = 0.5;
    ////double d_yaw = 0.0;
    //dr_prop.k = 1.8140e-04;
    //dr_prop.b = 4.5001e-06;
    //dr_prop.xypdis = 0.3785;
    //dr_prop.zpdis = 0.116;
    //dr_prop.Tscale = 10;
    //dr_prop.PIDlims = { 11.36, 11.36, 7.84, 1.57, 1.6, -9.81, 18.1, 2.0, 2.0, 1.0, 4267.0, 4927.1, 183.21 };
    //Eigen::Matrix3d drI;
    //drI << 93791.6663, -198.5303, 252.0061,
    //    -198.5303, 81226.6688, 2.8147,
    //    252.0061, 2.8147, 142817.7871;
    //dr_prop.In = drI/10000000;
    //dr_prop.kx = { 2.35, 0.0, 0.0 };
    //dr_prop.ky = { 2.25, 0.0, 0.0 };
    //dr_prop.kz = { 5.75, 0.4, 0.0 };
    //dr_prop.kvx = { 0.6, 0.0, 0.0 };
    //dr_prop.kvy = { -0.55, 0.0, 0.0};
    //dr_prop.kvz = { 5.85, 0.0, 0.1 };
    //dr_prop.kph = { 15.0, 1.5, 0.0 };
    //dr_prop.kth = { 15.0, 1.5, 0.0 };
    //dr_prop.kps = { 25.0, 0.0, 0.1 };
    //dr_prop.kdph = { 15.0, 1.5, 0.1 };
    //dr_prop.kdth = { 15.0, 1.5, 0.1 };
    //dr_prop.kdps = { 25.0, 0.1, 0.0 };

	/*ob_prop.g_p = { 0,0,0 };
	ob_prop.l = 0.5;
	ob_prop.w = 0.5;
	ob_prop.h = 0.5;
    Eigen::Matrix3d obI;
    obI << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;
    ob_prop.I = obI;
	ob_prop.mass = 1;
	ob_prop.con_rad = 1;   
    */

// frequency control for all the threads
double fcom = 100; // <= fsim    (25*, 50, 100, 200)
double fsim = 100;  // <= fcom2 (50*, 100, 200, 500)
double fcom2 = 1000; // <= frim  (100*, 200*, 500, 1000)
double fbrim = 1000; // <= fbmn  (500*, 1000, 2000, 5000*)
double fbmn = 1000; // (1000, 2000, 5000*)

// IRL drone frequencies
double fprim = 100;
double fcom3 = 50;
double fpx4 = 50;

// only used to update the visualization
double fvis = 60; // alaways 60 Hz

// whether IM uses ZOH(0), FOH(1), or RIM(2), first command line argument
int rim_type = 2; // (0, 1, 2*)

// file names for all the logging
string ADP_file = "";
string ADV_file = "";
string ADA_file = "";
string SDP_file = "";
string SDV_file = "";
string SDA_file = "";
string BDDP_file = "";
string BPDP_file = "";
string INTFOR_file = "";
string FREQ_file = "";
string BRtime_file = "";
string PRtime_file = "";
string contact_file = "";

// input vars for the MUS user to input (ID, Test#, Test_dir)
char ID_num[8] = "";
char Test_num[8] = "";
int Test_type = 0;

// experiment notes:
// rate Stability 0-10
// rate Torque Smoothness 0-10
// compare FOH and RIM to ZOH baseline results (considered 5/10 in both measures)

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
// global variables used to communicate between threads
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
// The visualization mesh to be used with polyscope (Eigen form)
Eigen::MatrixXd RoomV;
Eigen::MatrixXd RoomVOG;
Eigen::MatrixXi RoomF;

Eigen::MatrixXd DroneV;
Eigen::MatrixXd DroneVOG;
Eigen::MatrixXi DroneF;


// data logging for outputs (all in x,y,z)
int time_step = 0;
// logging files access vars
std::ofstream ADP_log_file;
std::ofstream ADV_log_file;
std::ofstream ADA_log_file;
std::ofstream SDP_log_file;
std::ofstream SDV_log_file;
std::ofstream SDA_log_file;
std::ofstream BDDP_log_file;
std::ofstream BPDP_log_file;
std::ofstream INTFOR_log_file;
std::ofstream FREQ_log_file;
std::ofstream BRtime_log_file;
std::ofstream PRtime_log_file;
std::ofstream contact_log_file;

// logging data vars for the real time it takes for the BRIM and PRIM to run
double BRtime = 0.0;
double BRcount = 0.0;
double PRtime = 0.0;
double PRcount = 0.0;

// global vars for loggin
// IRL drone stuff
Eigen::Vector3d ADP = Eigen::Vector3d::Zero();
Eigen::Vector3d ADV = Eigen::Vector3d::Zero();
Eigen::Vector3d ADA = Eigen::Vector3d::Zero();
// simulated drone stuff
Eigen::Vector3d SDP = Eigen::Vector3d::Zero();
Eigen::Vector3d SDV = Eigen::Vector3d::Zero();
Eigen::Vector3d SDA = Eigen::Vector3d::Zero();
// Bubble Method RIM stuff
Eigen::Vector3d BDDP = Eigen::Vector3d::Zero();
Eigen::Vector3d BPDP = Eigen::Vector3d::Zero();
Eigen::Vector3d IFOR = Eigen::Vector3d::Zero();
// Frequency data from all loops
Eigen::Matrix<double, 6, 1> FREQ = { 0,0,0,0,0,0 };


// initializing some global variables
BMN::Var_Transfer current_state;
//BMN::RIMdata RIM_data;
// vars used for visualization of the drone
Eigen::Vector3d Drone_pos = {0, 0, 1.0};
Eigen::Vector3d Drone_pos_pre = {0, 0, 1.0};
Eigen::Matrix3d Drone_RM = Eigen::Matrix3d::Identity();

// some helper bools for restarting the sim
bool reset_data_pipe = false;

// used for consistent start up order
bool vis_ready = false;
bool sim_ready = false;
bool bmn_ready = false;
bool brim_ready = false;
bool px4_ready = false;
bool prim_ready = false;
bool run = false;
bool run_check = false;
bool run_comm_loop = false;

bool wait = true;

bool start_loops = false;

bool irl_drone_ready = false;

// to ensure that the irl drones comms is only called once as it can't be "restarted" like the rest
bool first_run = true;