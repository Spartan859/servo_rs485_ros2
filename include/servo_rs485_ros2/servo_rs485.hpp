#pragma once
#include "servo_rs485_ros2/servo_base.hpp"
#include <string>
#include <cstdint>
#include <atomic>
#include <mutex>

/**
 * @brief RS485 总线舵机实现
 */
class ServoRS485 : public ServoBase {
public:
    ServoRS485(const std::string& port, int servo_id, int baudrate, double timeout);
    ~ServoRS485() override;

    bool ping() override;
    void setAngle(double degree, int time_ms = 0) override;
    void setAngleWithSpeed(double target_degree, double speed_dps = 30.0, 
                          int step_interval_ms = 50, std::atomic<bool>* cancel = nullptr) override;
    double getAngle() override;
    double currentDegree() override;
    std::string getType() const override { return "RS485"; }

private:
    int fd_;
    std::string port_;
    int servo_id_;
    int baudrate_;
    double timeout_;
    mutable std::mutex io_mutex_;

    // RS485 协议常量
    static constexpr int SERVO_MIN_POSITION = 0;
    static constexpr int SERVO_MAX_POSITION = 4096;

    bool openPort();
    void closePort();
    bool sendCommand(const uint8_t* data, size_t len, uint8_t* response, size_t& resp_len);
    uint8_t calculateChecksum(const uint8_t* data, size_t len);
    int degreeToPosition(double degree);
    double positionToDegree(int position);
};

// 保持向后兼容的别名
using Servo = ServoRS485;
