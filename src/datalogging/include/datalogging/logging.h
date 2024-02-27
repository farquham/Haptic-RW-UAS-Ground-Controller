#ifndef logging_H
#define logging_H

// include eigen dense and sparse
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <chrono>
#include <cstdio>
#include <mutex>

// ros2 msgs
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include "commsmsgs/msg/brimpub.hpp"
#include "commsmsgs/msg/bmnpub.hpp"
#include "commsmsgs/msg/rbquadsimpub.hpp"
#include "commsmsgs/msg/rrimpub.hpp"
#include "commsmsgs/msg/rpicommspub.hpp"
#include "commsmsgs/msg/logsetup.hpp"
#include "commsmsgs/msg/logctrlbools.hpp"

namespace datalogging {
	// class for offboard control which starts a ROS2 node
	class logging : public rclcpp::Node {
	public:
		logging() : Node("data_logging")
		{
            // setup subscriber for log file names
            name_logs_subscriber_ = this->create_subscription<commsmsgs::msg::Logsetup>("/GC/internal/logsetup", 10, std::bind(&logging::name_logs_callback, [this, brim_file, bmn_file, rbquadsim_file, rrim_file, rpi_file], std::placeholders::_1));

            // log control subscriber
            ctrl_logs_subscriber_ = this->create_subscription<commsmsgs::msg::Logctrlbools>("/GC/internal/logctrl", 10, std::bind(&logging::ctrl_logs_callback, [this, brim_file, brim_log_file, bmn_file, bmn_log_file, rbquadsim_file, rbquadsim_log_file, rrim_file, rrim_log_file, rpi_file, rpi_log_file], std::placeholders::_1));

			// state subscriber for recieving IRL drone position, velocity and acceleration
            brim_subscriber_ = this->create_subscription<commsmsgs::msg::brimPub>("/GC/out/brim", 10, std::bind(&logging::brim_callback, [this, brim_log_file], std::placeholders::_1));
            bmn_subscriber_ = this->create_subscription<commsmsgs::msg::bmnPub>("/GC/out/bmn", 10, std::bind(&logging::bmn_callback, [this, bmn_log_file], std::placeholders::_1));
            rbquadsim_subscriber_ = this->create_subscription<commsmsgs::msg::rbquadsimPub>("/GC/out/rbquadsim", 10, std::bind(&logging::rbquadsim_callback, [this, rbquadsim_log_file], std::placeholders::_1));
            rrim_subscriber_ = this->create_subscription<commsmsgs::msg::rrimPub>("/GC/out/rrim", 10, std::bind(&logging::rrim_callback, [this, rrim_log_file], std::placeholders::_1));
            rpi_subscriber_ = this->create_subscription<commsmsgs::msg::rpicommsPub>("/GC/out/rpi", 10, std::bind(&logging::rpi_callback, [this, rpi_log_file], std::placeholders::_1));
		}

	private:
        rclcpp::Subscription<commsmsgs::msg::Logsetup>::SharedPtr name_logs_subscriber_;
        rclcpp::Subscription<commsmsgs::msg::Logctrlbools>::SharedPtr ctrl_logs_subscriber_;

        rclcpp::Subscription<commsmsgs::msg::brimPub>::SharedPtr brim_subscriber_;
        rclcpp::Subscription<commsmsgs::msg::bmnPub>::SharedPtr bmn_subscriber_;
        rclcpp::Subscription<commsmsgs::msg::rbquadsimPub>::SharedPtr rbquadsim_subscriber_;
        rclcpp::Subscription<commsmsgs::msg::rrimPub>::SharedPtr rrim_subscriber_;
        rclcpp::Subscription<commsmsgs::msg::rpicommsPub>::SharedPtr rpi_subscriber_;

		std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

        std::string brim_file = "";
        std::ofstream brim_log_file;
        std::string bmn_file = "";
        std::ofstream bmn_log_file;
        std::string rbquadsim_file = "";
        std::ofstream rbquadsim_log_file;
        std::string rrim_file = "";
        std::ofstream rrim_log_file;
        std::string rpi_file = "";
        std::ofstream rpi_log_file;

		// series of functions needed to operate the subscribers
		void brim_callback(const commsmsgs::msg::brimPub::UniquePtr & msg, std::ofstream & log_file);
        void bmn_callback(const commsmsgs::msg::bmnPub::UniquePtr & msg, std::ofstream & log_file);
        void rbquadsim_callback(const commsmsgs::msg::rbquadsimPub::UniquePtr & msg, std::ofstream & log_file);
        void rrim_callback(const commsmsgs::msg::rrimPub::UniquePtr & msg, std::ofstream & log_file);
        void rpi_callback(const commsmsgs::msg::rpicommsPub::UniquePtr & msg, std::ofstream & log_file);

        // logging files helpers
        void name_log_file(std::string & file_name, int Part_ID, int exp_num, std::string type, int r_type, std::string file_type);
        void open_log_file(std::string file_name, std::ofstream & log_file);
        void close_log_file(std::ofstream & log_file);
        void clear_log_file(std::string file_name, std::ofstream & log_file);

        void name_logs_callback(const commsmsgs::msg::Logsetup::UniquePtr & msg, std::string & brim_file, std::string & bmn_file, std::string & rbquadsim_file, std::string & rrim_file, std::string & rpi_file);
        void ctrl_logs_callback(const commsmsgs::msg::Logctrlbools::UniquePtr & msg, std::string & brim_file, std::ofstream & brim_log_file, std::string & bmn_file, std::ofstream & bmn_log_file, std::string & rbquadsim_file, std::ofstream & rbquadsim_log_file, std::string & rrim_file, std::ofstream & rrim_log_file, std::string & rpi_file, std::ofstream & rpi_log_file);
	};
}

#endif