#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/srv/set_bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <boost/asio.hpp>
#include <thread>
#include <memory>
#include "ddsm115_driver/ddsm115_driver.hpp"

class DDSM115Node : public rclcpp::Node {
public:
    DDSM115Node() : Node("ddsm115_node"), io_context_(), work_guard_(io_context_.get_executor()) {
        // Declare parameters
        this->declare_parameter<std::string>("device_path", "/dev/ttyUSB0");
        this->declare_parameter<int>("motor_id", 1);
        this->declare_parameter<std::string>("motor_mode", "velocity");
        this->declare_parameter<double>("status_publish_rate", 10.0);
        this->declare_parameter<bool>("auto_polling", true);
        this->declare_parameter<int>("polling_interval_ms", 100);
        
        // Get parameters
        device_path_ = this->get_parameter("device_path").as_string();
        motor_id_ = static_cast<uint8_t>(this->get_parameter("motor_id").as_int());
        std::string mode_str = this->get_parameter("motor_mode").as_string();
        double status_rate = this->get_parameter("status_publish_rate").as_double();
        bool auto_polling = this->get_parameter("auto_polling").as_bool();
        int polling_interval = this->get_parameter("polling_interval_ms").as_int();
        
        // Convert mode string to enum
        if (mode_str == "current") {
            motor_mode_ = ddsm115::MotorMode::CURRENT_LOOP;
        } else if (mode_str == "velocity") {
            motor_mode_ = ddsm115::MotorMode::VELOCITY_LOOP;
        } else if (mode_str == "position") {
            motor_mode_ = ddsm115::MotorMode::POSITION_LOOP;
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown motor mode '%s', using velocity mode", mode_str.c_str());
            motor_mode_ = ddsm115::MotorMode::VELOCITY_LOOP;
        }
        
        // Initialize motor driver
        motor_driver_ = std::make_unique<ddsm115::DDSM115Driver>(io_context_, device_path_, motor_id_);
        
        // Set callbacks
        motor_driver_->set_error_callback([this](const std::string& error) {
            RCLCPP_ERROR(this->get_logger(), "Motor Error: %s", error.c_str());
        });
        
        motor_driver_->set_status_callback([this](const ddsm115::MotorStatus& status) {
            publish_status(status);
        });
        
        // Initialize motor
        if (!motor_driver_->initialize()) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize motor driver!");
            rclcpp::shutdown();
            return;
        }
        
        // Set motor mode
        if (!motor_driver_->set_mode(motor_mode_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set motor mode");
        }
        
        // Enable auto polling if requested
        if (auto_polling) {
            motor_driver_->set_auto_polling(true, polling_interval);
        }
        
        // Create publishers
        status_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("~/status", 10);
        
        // Create subscribers based on motor mode
        switch (motor_mode_) {
            case ddsm115::MotorMode::CURRENT_LOOP:
                current_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                    "~/current_command", 10,
                    std::bind(&DDSM115Node::current_callback, this, std::placeholders::_1));
                break;
                
            case ddsm115::MotorMode::VELOCITY_LOOP:
                velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                    "~/velocity_command", 10,
                    std::bind(&DDSM115Node::velocity_callback, this, std::placeholders::_1));
                break;
                
            case ddsm115::MotorMode::POSITION_LOOP:
                position_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                    "~/position_command", 10,
                    std::bind(&DDSM115Node::position_callback, this, std::placeholders::_1));
                break;
        }
        
        // Service for mode switching
        mode_service_ = this->create_service<std_msgs::srv::SetBool>(
            "~/set_mode", std::bind(&DDSM115Node::set_mode_callback, this, 
                                   std::placeholders::_1, std::placeholders::_2));
        
        // Timer for status publishing (if not using auto polling)
        if (!auto_polling) {
            status_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / status_rate)),
                std::bind(&DDSM115Node::status_timer_callback, this));
        }
        
        // Start io_context thread
        io_thread_ = std::thread([this]() {
            io_context_.run();
        });
        
        RCLCPP_INFO(this->get_logger(), "DDSM115 node initialized with motor ID %d on %s", 
                   motor_id_, device_path_.c_str());
    }
    
    ~DDSM115Node() {
        if (motor_driver_) {
            motor_driver_->shutdown();
        }
        
        work_guard_.reset();
        io_context_.stop();
        
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
    }

private:
    void current_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        int16_t current_ma = static_cast<int16_t>(msg->data * 1000.0);  // Convert A to mA
        if (!motor_driver_->set_current(current_ma)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set current to %d mA", current_ma);
        }
    }
    
    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        int16_t velocity_rpm = static_cast<int16_t>(msg->data);
        if (!motor_driver_->set_velocity(velocity_rpm)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set velocity to %d RPM", velocity_rpm);
        }
    }
    
    void position_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        double position_deg = msg->data;
        if (!motor_driver_->set_position(position_deg)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set position to %.2f degrees", position_deg);
        }
    }
    
    void publish_status(const ddsm115::MotorStatus& status) {
        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.header.stamp = this->now();
        joint_state.header.frame_id = "motor_" + std::to_string(motor_id_);
        
        // Fill joint state message
        joint_state.name.push_back("motor_joint");
        
        // Position (convert to radians)
        double position_rad = (static_cast<double>(status.position) * 2.0 * M_PI) / 32767.0;
        joint_state.position.push_back(position_rad);
        
        // Velocity (convert RPM to rad/s)
        double velocity_rad_s = (static_cast<double>(status.velocity) * 2.0 * M_PI) / 60.0;
        joint_state.velocity.push_back(velocity_rad_s);
        
        // Current (convert to Nm, approximate)
        double torque_nm = static_cast<double>(status.torque_current) * 0.00075;  // Using torque constant
        joint_state.effort.push_back(torque_nm);
        
        status_pub_->publish(joint_state);
        
        // Log errors if any
        if (status.error_code != 0) {
            std::string error_msg = "Motor errors: ";
            if (status.sensor_error()) error_msg += "SENSOR ";
            if (status.overcurrent_error()) error_msg += "OVERCURRENT ";
            if (status.phase_overcurrent_error()) error_msg += "PHASE_OVERCURRENT ";
            if (status.stall_error()) error_msg += "STALL ";
            if (status.troubleshooting()) error_msg += "TROUBLESHOOTING ";
            
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "%s", error_msg.c_str());
        }
    }
    
    void set_mode_callback(const std::shared_ptr<std_msgs::srv::SetBool::Request> request,
                          std::shared_ptr<std_msgs::srv::SetBool::Response> response) {
        // This is a placeholder - in a real implementation, you might want to switch modes
        // For now, just return the current mode status
        response->success = true;
        response->message = "Mode switching not implemented in this example";
    }
    
    void status_timer_callback() {
        if (!motor_driver_->request_status()) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                 "Failed to request motor status");
        }
    }

    // Member variables
    boost::asio::io_context io_context_;
    boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
    std::thread io_thread_;
    
    std::unique_ptr<ddsm115::DDSM115Driver> motor_driver_;
    std::string device_path_;
    uint8_t motor_id_;
    ddsm115::MotorMode motor_mode_;
    
    // ROS2 interfaces
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr status_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr current_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_sub_;
    rclcpp::Service<std_msgs::srv::SetBool>::SharedPtr mode_service_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<DDSM115Node>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
