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
#include "commsmsgs/msg/guicontrols.hpp"

namespace datalogging {
	// class for offboard control which starts a ROS2 node
	class logging : public rclcpp::Node {
	public:
		logging() : Node("data_logging")
		{
            // setup subscriber for log file names
            name_logs_subscriber_ = this->create_subscription<commsmsgs::msg::Logsetup>("/GC/internal/logsetup", 10, std::bind(&logging::name_logs_callback, [this, brim_file, bmn_file, rbquadsim_file, rrim_file, rpi_file], std::placeholders::_1));

            // control subscriber
            ctrl_subscriber_ = this->create_subscription<commsmsgs::msg::Guicontrols>("/GC/internal/guictrls", 10, std::bind(&logging::ctrl_callback, [this, brim_file, brim_log_file, bmn_file, bmn_log_file, rbquadsim_file, rbquadsim_log_file, rrim_file, rrim_log_file, rpi_file, rpi_log_file], std::placeholders::_1));

            // log state publisher
            logs_publisher_ = this->create_publisher<commsmsgs::msg::Logctrlbools>("/GC/internal/logctrl", 10);

            auto timer_callback = [this]() -> void {
                // what ever code to run every timer iteration
                this->publish_log_state();
            };

            timer_ = this->create_wall_timer(100ms, timer_callback);
            if self.logs_open {
                // state subscriber for recieving IRL drone position, velocity and acceleration
                brim_subscriber_ = this->create_subscription<commsmsgs::msg::Brimpub>("/GC/out/brim", 10, std::bind(&logging::brim_callback, [this, brim_log_file], std::placeholders::_1));
                bmn_subscriber_ = this->create_subscription<commsmsgs::msg::Bmnpub>("/GC/out/bmn", 10, std::bind(&logging::bmn_callback, [this, bmn_log_file], std::placeholders::_1));
                rbquadsim_subscriber_ = this->create_subscription<commsmsgs::msg::Rbquadsimpub>("/GC/out/rbquadsim", 10, std::bind(&logging::rbquadsim_callback, [this, rbquadsim_log_file], std::placeholders::_1));
                rrim_subscriber_ = this->create_subscription<commsmsgs::msg::Rrimpub>("/GC/out/prim", 10, std::bind(&logging::rrim_callback, [this, rrim_log_file], std::placeholders::_1));
                rpi_subscriber_ = this->create_subscription<commsmsgs::msg::Rpicommspub>("/GC/out/rpicomms", 10, std::bind(&logging::rpi_callback, [this, rpi_log_file], std::placeholders::_1));
            }
		}

	private:
        rclcpp::Subscription<commsmsgs::msg::Logsetup>::SharedPtr name_logs_subscriber_;
        rclcpp::Subscription<commsmsgs::msg::Guicontrols>::SharedPtr ctrl_subscriber_;
        rclcpp::Publisher<commsmsgs::msg::Logctrlbools>::SharedPtr logs_publisher_;

        rclcpp::Subscription<commsmsgs::msg::Brimpub>::SharedPtr brim_subscriber_;
        rclcpp::Subscription<commsmsgs::msg::Bmnpub>::SharedPtr bmn_subscriber_;
        rclcpp::Subscription<commsmsgs::msg::Rbquadsimpub>::SharedPtr rbquadsim_subscriber_;
        rclcpp::Subscription<commsmsgs::msg::Rrimpub>::SharedPtr rrim_subscriber_;
        rclcpp::Subscription<commsmsgs::msg::Rpicommspub>::SharedPtr rpi_subscriber_;

        rclcpp::TimerBase::SharedPtr timer_;

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

        bool logs_open = false;
        bool logs_closed = false;
        bool logs_cleared = false;

		// series of functions needed to operate the subscribers
		void brim_callback(const commsmsgs::msg::Brimpub::UniquePtr & msg, std::ofstream & log_file);
        void bmn_callback(const commsmsgs::msg::Bmnpub::UniquePtr & msg, std::ofstream & log_file);
        void rbquadsim_callback(const commsmsgs::msg::Rbquadsimpub::UniquePtr & msg, std::ofstream & log_file);
        void rrim_callback(const commsmsgs::msg::Rrimpub::UniquePtr & msg, std::ofstream & log_file);
        void rpi_callback(const commsmsgs::msg::Rpicommspub::UniquePtr & msg, std::ofstream & log_file);

        void publish_log_state();

        // logging files helpers
        void name_log_file(std::string & file_name, int Part_ID, int exp_num, std::string type, int r_type, std::string file_type);
        void open_log_file(std::string file_name, std::ofstream & log_file);
        void close_log_file(std::ofstream & log_file);
        void clear_log_file(std::string file_name, std::ofstream & log_file);

        void name_logs_callback(const commsmsgs::msg::Logsetup::UniquePtr & msg, std::string & brim_file, std::string & bmn_file, std::string & rbquadsim_file, std::string & rrim_file, std::string & rpi_file);
        void ctrl_callback(const commsmsgs::msg::Guicontrols::UniquePtr & msg, std::string & brim_file, std::ofstream & brim_log_file, std::string & bmn_file, std::ofstream & bmn_log_file, std::string & rbquadsim_file, std::ofstream & rbquadsim_log_file, std::string & rrim_file, std::ofstream & rrim_log_file, std::string & rpi_file, std::ofstream & rpi_log_file);
	};
}

#endif