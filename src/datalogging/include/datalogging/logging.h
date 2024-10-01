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
#include <rosbag2_cpp/writer.hpp>
#include <rclcpp/serialization.hpp>
#include <stdint.h>
#include "commsmsgs/msg/brimpub.hpp"
#include "commsmsgs/msg/bmnpub.hpp"
#include "commsmsgs/msg/rbquadsimpub.hpp"
#include "commsmsgs/msg/rrimpub.hpp"
#include "commsmsgs/msg/rpicommspub.hpp"
#include "commsmsgs/msg/logsetup.hpp"
#include "commsmsgs/msg/logctrlbools.hpp"
#include "commsmsgs/msg/guicontrols.hpp"

typedef std::chrono::high_resolution_clock clocky;
using namespace std::chrono_literals;

namespace datalogging {
	// class for offboard control which starts a ROS2 node
	class logging : public rclcpp::Node {
	public:
		logging() : Node("data_logging")
		{
            // setup rosbag writer
            writer_ = std::make_unique<rosbag2_cpp::Writer>();
            auto time = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(time);
            std::stringstream date_out;
            date_out << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H-%M-%S");
            auto save_location = "DataLog/" + date_out.str();
            RCLCPP_INFO(this->get_logger(), "Saving logs to: %s", save_location.c_str());
            writer_->open(save_location);

            // control subscriber
            ctrl_subscriber_ = this->create_subscription<commsmsgs::msg::Guicontrols>("/GC/internal/guictrls", 10, std::bind(&logging::ctrl_logs_callback, this, std::placeholders::_1));
            logsetup_subscriber_ = this->create_subscription<commsmsgs::msg::Logsetup>("/GC/internal/logsetup", 10, std::bind(&logging::logsetup_callback, this, std::placeholders::_1));

            // log state publisher
            logs_publisher_ = this->create_publisher<commsmsgs::msg::Logctrlbools>("/GC/internal/logctrl", 10);

            timer_ = this->create_wall_timer(100ms, std::bind(&datalogging::logging::timer_callback, this));

            // state subscriber for recieving IRL drone position, velocity and acceleration
            brim_subscriber_ = this->create_subscription<commsmsgs::msg::Brimpub>("/GC/out/brim", 10, std::bind(&logging::brim_callback, this, std::placeholders::_1));
            bmn_subscriber_ = this->create_subscription<commsmsgs::msg::Bmnpub>("/GC/out/bmn", 10, std::bind(&logging::bmn_callback, this, std::placeholders::_1));
            rbquadsim_subscriber_ = this->create_subscription<commsmsgs::msg::Rbquadsimpub>("/GC/out/rbquadsim", 10, std::bind(&logging::rbquadsim_callback, this, std::placeholders::_1));
            rrim_subscriber_ = this->create_subscription<commsmsgs::msg::Rrimpub>("/GC/out/prim", 10, std::bind(&logging::rrim_callback, this, std::placeholders::_1));
            rpi_subscriber_ = this->create_subscription<commsmsgs::msg::Rpicommspub>("/GC/out/rpicomms", 10, std::bind(&logging::rpi_callback, this, std::placeholders::_1));
		}

	private:
        void timer_callback() {
            this->publish_log_state();
        }
        rclcpp::Subscription<commsmsgs::msg::Logsetup>::SharedPtr name_logs_subscriber_;
        rclcpp::Subscription<commsmsgs::msg::Guicontrols>::SharedPtr ctrl_subscriber_;
        rclcpp::Publisher<commsmsgs::msg::Logctrlbools>::SharedPtr logs_publisher_;
        rclcpp::Subscription<commsmsgs::msg::Logsetup>::SharedPtr logsetup_subscriber_;

        rclcpp::Subscription<commsmsgs::msg::Brimpub>::SharedPtr brim_subscriber_;
        rclcpp::Subscription<commsmsgs::msg::Bmnpub>::SharedPtr bmn_subscriber_;
        rclcpp::Subscription<commsmsgs::msg::Rbquadsimpub>::SharedPtr rbquadsim_subscriber_;
        rclcpp::Subscription<commsmsgs::msg::Rrimpub>::SharedPtr rrim_subscriber_;
        rclcpp::Subscription<commsmsgs::msg::Rpicommspub>::SharedPtr rpi_subscriber_;

        rclcpp::TimerBase::SharedPtr timer_;

        std::unique_ptr<rosbag2_cpp::Writer> writer_;

		std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

        bool logs_open = false;
        bool logs_closed = true;
        bool logs_cleared = false;

        bool sim_running = false;
        bool brim_running = false;
        bool bmn_running = false;
        bool rpicomms_running = false;
        bool rrim_running = false;

		// series of functions needed to operate the subscribers
		void brim_callback(std::shared_ptr<rclcpp::SerializedMessage> message);
        void bmn_callback(std::shared_ptr<rclcpp::SerializedMessage> message);
        void rbquadsim_callback(std::shared_ptr<rclcpp::SerializedMessage> message);
        void rrim_callback(std::shared_ptr<rclcpp::SerializedMessage> message);
        void rpi_callback(std::shared_ptr<rclcpp::SerializedMessage> message);

        void logsetup_callback(std::shared_ptr<rclcpp::SerializedMessage> message);

        void publish_log_state();

        void ctrl_logs_callback(const commsmsgs::msg::Guicontrols::UniquePtr & msg);
	};
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<datalogging::logging>());
    rclcpp::shutdown();
    return 0;
}

#endif