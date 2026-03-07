#include "servo_rs485_ros2/servo_pwm.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <array>
#include <memory>
#include <chrono>
#include <thread>
#include <sstream>
#include <iomanip>

ServoPWM::ServoPWM(uint32_t pwm_period,
                   uint32_t duty_min,
                   uint32_t duty_max,
                   uint64_t reg_mux_addr,
                   uint64_t reg_period_addr,
                   uint64_t reg_low_addr,
                   uint64_t reg_high_addr)
    : pwm_period_(pwm_period),
      duty_min_(duty_min), 
      duty_max_(duty_max),
      reg_mux_addr_(reg_mux_addr),
      reg_period_addr_(reg_period_addr),
      reg_low_addr_(reg_low_addr),
      reg_high_addr_(reg_high_addr) {
    last_angle_ = 0.0;
    initialized_ = initPWM();
    if (!initialized_) {
        RCLCPP_WARN(rclcpp::get_logger("ServoPWM"), 
                    "PWM initialization failed. Check devmem permissions and register addresses.");
    }
}

ServoPWM::~ServoPWM() {
    // PWM 硬件无需特殊清理
}

bool ServoPWM::initPWM() {
    // 1. 设置 PWM 引脚复用
    if (!writeRegister(reg_mux_addr_, 0)) {
        RCLCPP_ERROR(rclcpp::get_logger("ServoPWM"), 
                     "Failed to set PWM pin mux at 0x%llX", 
                     static_cast<unsigned long long>(reg_mux_addr_));
        return false;
    }

    // 2. 设置 PWM 周期
    if (!writeRegister(reg_period_addr_, pwm_period_)) {
        RCLCPP_ERROR(rclcpp::get_logger("ServoPWM"), 
                     "Failed to set PWM period at 0x%llX", 
                     static_cast<unsigned long long>(reg_period_addr_));
        return false;
    }

    // 3. 设置 PWM 低电平开始时刻为 0
    if (!writeRegister(reg_low_addr_, 0)) {
        RCLCPP_ERROR(rclcpp::get_logger("ServoPWM"), 
                     "Failed to set PWM low start at 0x%llX", 
                     static_cast<unsigned long long>(reg_low_addr_));
        return false;
    }

    // 4. 设置初始角度为 0 度（中间位置）
    uint32_t center_duty = degreeToDuty(0.0);
    if (!writeRegister(reg_high_addr_, center_duty)) {
        RCLCPP_ERROR(rclcpp::get_logger("ServoPWM"), 
                     "Failed to set initial PWM duty at 0x%llX", 
                     static_cast<unsigned long long>(reg_high_addr_));
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("ServoPWM"), 
                "PWM servo initialized: period=%u, duty_range=[%u, %u], center_duty=%u",
                pwm_period_, duty_min_, duty_max_, center_duty);
    return true;
}

bool ServoPWM::writeRegister(uint64_t addr, uint32_t value) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    
    // 构建 devmem 命令
    std::ostringstream cmd;
    cmd << "devmem 0x" << std::hex << std::uppercase << std::setfill('0') 
        << std::setw(10) << addr << " w " << std::dec << value << " 2>/dev/null";
    
    int ret = std::system(cmd.str().c_str());
    if (ret != 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("ServoPWM"), 
                     "devmem write failed: %s (ret=%d)", cmd.str().c_str(), ret);
        return false;
    }
    return true;
}

bool ServoPWM::readRegister(uint64_t addr, uint32_t& value) {
    std::lock_guard<std::mutex> lock(io_mutex_);
    
    // 构建 devmem 读取命令
    std::ostringstream cmd;
    cmd << "devmem 0x" << std::hex << std::uppercase << std::setfill('0') 
        << std::setw(10) << addr << " 2>/dev/null";
    
    // 使用 popen 读取命令输出
    std::array<char, 128> buffer;
    std::string result;
    
    FILE* pipe = popen(cmd.str().c_str(), "r");
    if (!pipe) {
        RCLCPP_DEBUG(rclcpp::get_logger("ServoPWM"), 
                     "popen failed for: %s", cmd.str().c_str());
        return false;
    }
    
    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
        result += buffer.data();
    }
    
    int ret = pclose(pipe);
    if (ret != 0 || result.empty()) {
        RCLCPP_DEBUG(rclcpp::get_logger("ServoPWM"), 
                     "devmem read failed: %s", cmd.str().c_str());
        return false;
    }
    
    // 解析输出（devmem 输出格式为 "0xVALUE\n"）
    try {
        value = static_cast<uint32_t>(std::stoul(result, nullptr, 0));
        return true;
    } catch (const std::exception& e) {
        RCLCPP_DEBUG(rclcpp::get_logger("ServoPWM"), 
                     "Failed to parse devmem output: %s", result.c_str());
        return false;
    }
}

uint32_t ServoPWM::degreeToDuty(double degree) {
    degree = clampDegree(degree);
    double deg_range = SERVO_MAX_DEGREE - SERVO_MIN_DEGREE;
    double duty_range = static_cast<double>(duty_max_ - duty_min_);
    double duty = duty_min_ + ((degree - SERVO_MIN_DEGREE) / deg_range) * duty_range;
    return static_cast<uint32_t>(duty);
}

double ServoPWM::dutyToDegree(uint32_t duty) {
    if (duty < duty_min_) duty = duty_min_;
    if (duty > duty_max_) duty = duty_max_;
    double deg_range = SERVO_MAX_DEGREE - SERVO_MIN_DEGREE;
    double duty_range = static_cast<double>(duty_max_ - duty_min_);
    double degree = SERVO_MIN_DEGREE + ((duty - duty_min_) / duty_range) * deg_range;
    return degree;
}

bool ServoPWM::ping() {
    // PWM 舵机无法真正探测，只检查初始化状态
    // 尝试读取当前占空比寄存器作为简单检测
    if (!initialized_) {
        return false;
    }
    
    uint32_t dummy;
    return readRegister(reg_high_addr_, dummy);
}

void ServoPWM::setAngle(double degree, int time_ms) {
    (void)time_ms;  // PWM 舵机不支持时间参数，忽略
    
    if (!initialized_) {
        RCLCPP_WARN(rclcpp::get_logger("ServoPWM"), 
                    "PWM not initialized, cannot set angle");
        return;
    }
    
    uint32_t duty = degreeToDuty(degree);
    if (writeRegister(reg_high_addr_, duty)) {
        last_angle_ = clampDegree(degree);
        RCLCPP_DEBUG(rclcpp::get_logger("ServoPWM"), 
                     "Set angle: %.2f deg -> duty: %u", degree, duty);
    } else {
        RCLCPP_WARN(rclcpp::get_logger("ServoPWM"), 
                    "Failed to set angle: %.2f deg", degree);
    }
}

double ServoPWM::getAngle() {
    if (!initialized_) {
        return last_angle_;
    }
    
    uint32_t duty;
    if (readRegister(reg_high_addr_, duty)) {
        last_angle_ = dutyToDegree(duty);
    }
    return last_angle_;
}

double ServoPWM::currentDegree() {
    return getAngle();
}

void ServoPWM::setAngleWithSpeed(double target_degree, double speed_dps, 
                                  int step_interval_ms, std::atomic<bool>* cancel) {
    if (speed_dps <= 0.0) {
        // 直接跳转到目标
        setAngle(target_degree, step_interval_ms);
        return;
    }
    
    if (step_interval_ms < 1) step_interval_ms = 10;
    
    double current_deg = currentDegree();
    double angle_diff = target_degree - current_deg;
    
    if (std::abs(angle_diff) < 0.5) {
        // 角度差很小，直接设置
        setAngle(target_degree, step_interval_ms);
        return;
    }
    
    // 计算步进
    double total_time_s = std::abs(angle_diff) / speed_dps;
    int total_time_ms = static_cast<int>(total_time_s * 1000.0);
    int num_steps = std::max(1, total_time_ms / step_interval_ms);
    double step_deg = angle_diff / static_cast<double>(num_steps);
    
    for (int i = 1; i <= num_steps; ++i) {
        if (cancel && cancel->load()) {
            return;  // 被取消
        }
        double intermediate_deg = current_deg + step_deg * i;
        setAngle(intermediate_deg, step_interval_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(step_interval_ms));
    }
    
    // 确保最终位置
    if (cancel && cancel->load()) {
        return;
    }
    setAngle(target_degree, step_interval_ms);
}
