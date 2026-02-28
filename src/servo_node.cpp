#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "servo_rs485_ros2/servo_base.hpp"
#include "servo_rs485_ros2/servo_rs485.hpp"
#include "servo_rs485_ros2/servo_pwm.hpp"
#include "servo_rs485_ros2/srv/ping_servo.hpp"
#include "servo_rs485_ros2/srv/set_angle.hpp"
#include "servo_rs485_ros2/srv/get_angle.hpp"
#include "servo_rs485_ros2/srv/set_angle_with_speed.hpp"
#include <memory>
#include <atomic>
#include <thread>

class ServoNode : public rclcpp::Node {
public:
    ServoNode() : Node("servo_rs485_node") {
        // 通用参数
        servo_type_ = this->declare_parameter<std::string>("servo_type", "rs485");
        publish_interval_ms_ = this->declare_parameter<int>("publish_interval_ms", 30);

        // RS485 舵机参数
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB1");
        servo_id_ = this->declare_parameter<int>("servo_id", 1);
        baudrate_ = this->declare_parameter<int>("baudrate", 115200);
        timeout_ = this->declare_parameter<double>("timeout", 0.5);

        // PWM 舵机参数
        pwm_period_ = this->declare_parameter<int>("pwm_period", 10000);
        pwm_duty_min_ = this->declare_parameter<int>("pwm_duty_min", 250);
        pwm_duty_max_ = this->declare_parameter<int>("pwm_duty_max", 1250);
        // 寄存器地址（使用字符串形式以支持十六进制输入）
        std::string reg_mux_str = this->declare_parameter<std::string>("pwm_reg_mux", "0x00C40000D0");
        std::string reg_period_str = this->declare_parameter<std::string>("pwm_reg_period", "0x00C408002C");
        std::string reg_low_str = this->declare_parameter<std::string>("pwm_reg_low", "0x00C4080030");
        std::string reg_high_str = this->declare_parameter<std::string>("pwm_reg_high", "0x00C4080034");

        // 创建舵机实例
        if (servo_type_ == "pwm") {
            auto reg_mux = static_cast<uint64_t>(std::stoull(reg_mux_str, nullptr, 0));
            auto reg_period = static_cast<uint64_t>(std::stoull(reg_period_str, nullptr, 0));
            auto reg_low = static_cast<uint64_t>(std::stoull(reg_low_str, nullptr, 0));
            auto reg_high = static_cast<uint64_t>(std::stoull(reg_high_str, nullptr, 0));
            
            servo_ = std::make_shared<ServoPWM>(
                static_cast<uint32_t>(pwm_period_),
                static_cast<uint32_t>(pwm_duty_min_),
                static_cast<uint32_t>(pwm_duty_max_),
                reg_mux, reg_period, reg_low, reg_high
            );
            RCLCPP_INFO(this->get_logger(), "Using PWM servo (period=%d, duty=[%d, %d])",
                        pwm_period_, pwm_duty_min_, pwm_duty_max_);
        } else {
            servo_ = std::make_shared<ServoRS485>(port_, servo_id_, baudrate_, timeout_);
            RCLCPP_INFO(this->get_logger(), "Using RS485 servo (port=%s, id=%d)",
                        port_.c_str(), servo_id_);
        }

        srv_ping_ = this->create_service<servo_rs485_ros2::srv::PingServo>(
            "ping_servo", std::bind(&ServoNode::ping_callback, this, std::placeholders::_1, std::placeholders::_2));
        srv_set_angle_ = this->create_service<servo_rs485_ros2::srv::SetAngle>(
            "set_angle", std::bind(&ServoNode::set_angle_callback, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_angle_ = this->create_service<servo_rs485_ros2::srv::GetAngle>(
            "get_angle", std::bind(&ServoNode::get_angle_callback, this, std::placeholders::_1, std::placeholders::_2));
        srv_set_angle_speed_ = this->create_service<servo_rs485_ros2::srv::SetAngleWithSpeed>(
            "set_angle_with_speed", std::bind(&ServoNode::set_angle_with_speed_callback, this, std::placeholders::_1, std::placeholders::_2));
        angle_pub_ = this->create_publisher<std_msgs::msg::Float64>("servo_angle", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_interval_ms_), std::bind(&ServoNode::publish_angle, this));
    }
    ~ServoNode() override {
        cancel_.store(true);
        if (worker_.joinable()) {
            worker_.join();
        }
    }
private:
    void ping_callback(const std::shared_ptr<servo_rs485_ros2::srv::PingServo::Request> req,
                      std::shared_ptr<servo_rs485_ros2::srv::PingServo::Response> res) {
        res->online = servo_->ping();
    }
    void set_angle_callback(const std::shared_ptr<servo_rs485_ros2::srv::SetAngle::Request> req,
                           std::shared_ptr<servo_rs485_ros2::srv::SetAngle::Response> res) {
        servo_->setAngle(req->degree, req->time_ms);
        res->success = true;
    }
    void get_angle_callback(const std::shared_ptr<servo_rs485_ros2::srv::GetAngle::Request> req,
                           std::shared_ptr<servo_rs485_ros2::srv::GetAngle::Response> res) {
        double deg = servo_->currentDegree();
        res->degree = deg;
        res->success = true;
    }
    void set_angle_with_speed_callback(const std::shared_ptr<servo_rs485_ros2::srv::SetAngleWithSpeed::Request> req,
                                       std::shared_ptr<servo_rs485_ros2::srv::SetAngleWithSpeed::Response> res) {
        // Preempt: cancel previous motion and join any running worker
        cancel_.store(true);
        if (worker_.joinable()) {
            worker_.join();
        }
        moving_.store(false);
        cancel_.store(false);
        moving_.store(true);
        double degree = req->degree;
        double speed = req->speed_dps;
        int step = req->step_interval_ms;
        worker_ = std::thread([this, degree, speed, step]() {
            try {
                servo_->setAngleWithSpeed(degree, speed, step, &cancel_);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Worker thread exception: %s", e.what());
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "Worker thread unknown exception");
            }
            moving_.store(false);
        });
        res->success = true;
    }
    void publish_angle() {
        double deg = servo_->currentDegree();
        auto msg = std_msgs::msg::Float64();
        msg.data = deg;
        angle_pub_->publish(msg);
    }

    // 通用参数
    std::string servo_type_;
    int publish_interval_ms_;

    // RS485 舵机参数
    std::string port_;
    int servo_id_;
    int baudrate_;
    double timeout_;

    // PWM 舵机参数
    int pwm_period_;
    int pwm_duty_min_;
    int pwm_duty_max_;

    // 舵机实例（基类指针）
    std::shared_ptr<ServoBase> servo_;

    rclcpp::Service<servo_rs485_ros2::srv::PingServo>::SharedPtr srv_ping_;
    rclcpp::Service<servo_rs485_ros2::srv::SetAngle>::SharedPtr srv_set_angle_;
    rclcpp::Service<servo_rs485_ros2::srv::GetAngle>::SharedPtr srv_get_angle_;
    rclcpp::Service<servo_rs485_ros2::srv::SetAngleWithSpeed>::SharedPtr srv_set_angle_speed_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<bool> moving_{false};
    std::thread worker_;
    std::atomic<bool> cancel_{false};
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
