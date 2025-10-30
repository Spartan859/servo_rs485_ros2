#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "servo_rs485_ros2/servo_rs485.hpp"
#include "servo_rs485_ros2/srv/ping_servo.hpp"
#include "servo_rs485_ros2/srv/set_angle.hpp"
#include "servo_rs485_ros2/srv/get_angle.hpp"
#include <memory>

class ServoNode : public rclcpp::Node {
public:
    ServoNode() : Node("servo_rs485_node") {
        port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB1");
        servo_id_ = this->declare_parameter<int>("servo_id", 1);
        baudrate_ = this->declare_parameter<int>("baudrate", 115200);
        timeout_ = this->declare_parameter<double>("timeout", 0.5);
        servo_ = std::make_shared<Servo>(port_, servo_id_, baudrate_, timeout_);
        srv_ping_ = this->create_service<servo_rs485_ros2::srv::PingServo>(
            "ping_servo", std::bind(&ServoNode::ping_callback, this, std::placeholders::_1, std::placeholders::_2));
        srv_set_angle_ = this->create_service<servo_rs485_ros2::srv::SetAngle>(
            "set_angle", std::bind(&ServoNode::set_angle_callback, this, std::placeholders::_1, std::placeholders::_2));
        srv_get_angle_ = this->create_service<servo_rs485_ros2::srv::GetAngle>(
            "get_angle", std::bind(&ServoNode::get_angle_callback, this, std::placeholders::_1, std::placeholders::_2));
        angle_pub_ = this->create_publisher<std_msgs::msg::Float64>("servo_angle", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ServoNode::publish_angle, this));
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
    void publish_angle() {
        double deg = servo_->currentDegree();
        auto msg = std_msgs::msg::Float64();
        msg.data = deg;
        angle_pub_->publish(msg);
    }
    std::string port_;
    int servo_id_;
    int baudrate_;
    double timeout_;
    std::shared_ptr<Servo> servo_;
    rclcpp::Service<servo_rs485_ros2::srv::PingServo>::SharedPtr srv_ping_;
    rclcpp::Service<servo_rs485_ros2::srv::SetAngle>::SharedPtr srv_set_angle_;
    rclcpp::Service<servo_rs485_ros2::srv::GetAngle>::SharedPtr srv_get_angle_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
