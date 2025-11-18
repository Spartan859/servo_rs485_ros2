#pragma once
#include <string>
#include <cstdint>

class Servo {
public:
    Servo(const std::string& port, int servo_id, int baudrate, double timeout);
    bool ping();
    void setAngle(double degree, int time_ms = 0);
    void setAngleWithSpeed(double target_degree, double speed_dps = 30.0, int step_interval_ms = 50);
    double getAngle();
    double currentDegree();
private:
    int fd_;
    std::string port_;
    int servo_id_;
    int baudrate_;
    double timeout_;
    double last_angle_;
    bool openPort();
    void closePort();
    bool sendCommand(const uint8_t* data, size_t len, uint8_t* response, size_t& resp_len);
    uint8_t calculateChecksum(const uint8_t* data, size_t len);
    int degreeToPosition(double degree);
    double positionToDegree(int position);
};
