#include "servo_rs485_ros2/servo_rs485.hpp"
#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>

#define HEADER1 0x12
#define HEADER2 0x4C
#define CMD_PING 0x01
#define CMD_READ_DATA 0x02
#define CMD_WRITE_DATA 0x03
#define ADDR_SET_POSITION_TIME 0x2A
#define ADDR_GET_POSITION 0x38
#define SERVO_MIN_POSITION 0
#define SERVO_MAX_POSITION 4096
#define SERVO_MIN_DEGREE -135.0
#define SERVO_MAX_DEGREE 135.0

Servo::Servo(const std::string& port, int servo_id, int baudrate, double timeout)
    : port_(port), servo_id_(servo_id), baudrate_(baudrate), timeout_(timeout), fd_(-1), last_angle_(0.0) {
    openPort();
}

bool Servo::openPort() {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) return false;
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd_, &tty) != 0) return false;
    cfsetospeed(&tty, baudrate_);
    cfsetispeed(&tty, baudrate_);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = static_cast<int>(timeout_ * 10);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) return false;
    return true;
}

void Servo::closePort() {
    if (fd_ >= 0) close(fd_);
    fd_ = -1;
}

uint8_t Servo::calculateChecksum(const uint8_t* data, size_t len) {
    int sum = 0;
    for (size_t i = 0; i < len; ++i) sum += data[i];
    return (~sum) & 0xFF;
}

bool Servo::sendCommand(const uint8_t* data, size_t len, uint8_t* response, size_t& resp_len) {
    if (fd_ < 0) return false;
    write(fd_, data, len);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    resp_len = read(fd_, response, 32);
    // print raw response
    // std::ostringstream oss;
    // for (size_t i = 0; i < resp_len; ++i) {
    //     oss << static_cast<int>(response[i]);
    //     if (i < resp_len - 1) oss << ", ";
    // }
    // RCLCPP_INFO(rclcpp::get_logger("ServoDriver"), "Raw Response: [%s]", oss.str().c_str());

    if (resp_len > 0) {
        // 查找与本机ID匹配的帧头（响应通常直接以ID开头，无12 4C）
        size_t start_index = resp_len;
        for (size_t i = 0; i < resp_len; ++i) {
            if (response[i] == servo_id_) {
                start_index = i;
                break;
            }
        }
        if (start_index < resp_len) {
            // 截取从正确ID开始的有效帧
            size_t valid_len = resp_len - start_index;
            memmove(response, response + start_index, valid_len);
            resp_len = valid_len;
        } else {
            // 没有找到ID，响应无效
            resp_len = 0;
        }
    }
    return resp_len > 0;
}

bool Servo::ping() {
    uint8_t cmd[6];
    cmd[0] = HEADER1;
    cmd[1] = HEADER2;
    cmd[2] = servo_id_;
    cmd[3] = 2;
    cmd[4] = CMD_PING;
    cmd[5] = calculateChecksum(&cmd[2], 3); // 只对ID, LEN, CMD计算
    uint8_t resp[32];
    size_t resp_len = 0;
    return sendCommand(cmd, 6, resp, resp_len);
}

void Servo::setAngle(double degree, int time_ms) {
    int position = degreeToPosition(degree);
    int time_val = std::min(time_ms << 2, 0xFFFF);
    uint8_t data_to_write[4];
    data_to_write[0] = (position >> 8) & 0xFF;
    data_to_write[1] = position & 0xFF;
    data_to_write[2] = (time_val >> 8) & 0xFF;
    data_to_write[3] = time_val & 0xFF;
    uint8_t cmd[11];
    cmd[0] = HEADER1;
    cmd[1] = HEADER2;
    cmd[2] = servo_id_;
    cmd[3] = 7;
    cmd[4] = CMD_WRITE_DATA;
    cmd[5] = ADDR_SET_POSITION_TIME;
    memcpy(&cmd[6], data_to_write, 4);
    cmd[10] = calculateChecksum(&cmd[2], 8);
    uint8_t resp[32];
    size_t resp_len = 0;
    sendCommand(cmd, 11, resp, resp_len);
}

void Servo::setAngleWithSpeed(double target_degree, double speed_dps, int step_interval_ms) {
    // 1. 读取当前角度
    double current_deg = currentDegree();
    if (std::isnan(current_deg)) {
        current_deg = last_angle_;
    }

    // 2. 计算角度差与总时间
    double angle_diff = target_degree - current_deg;
    if (std::abs(angle_diff) < 0.5) {
        // 差异过小，直接到位
        setAngle(target_degree, step_interval_ms);
        return;
    }

    double total_time_s = std::abs(angle_diff) / speed_dps;
    int total_time_ms = static_cast<int>(total_time_s * 1000);

    // 3. 拆分成若干步
    int num_steps = std::max(1, total_time_ms / step_interval_ms);
    double step_deg = angle_diff / num_steps;

    // 4. 循环执行每步
    for (int i = 1; i <= num_steps; ++i) {
        double intermediate_deg = current_deg + step_deg * i;
        setAngle(intermediate_deg, step_interval_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(step_interval_ms));
    }

    // 5. 最后确保精确到位
    setAngle(target_degree, step_interval_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(step_interval_ms));
    last_angle_ = target_degree;
}

double Servo::getAngle() {
    uint8_t cmd[8];
    cmd[0] = HEADER1;
    cmd[1] = HEADER2;
    cmd[2] = servo_id_;
    cmd[3] = 4;
    cmd[4] = CMD_READ_DATA;
    cmd[5] = ADDR_GET_POSITION;
    cmd[6] = 2;
    cmd[7] = calculateChecksum(&cmd[2], 6);
    uint8_t resp[32];
    size_t resp_len = 0;

    if (sendCommand(cmd, 8, resp, resp_len)) { // 只要有响应就进入
        
        // RCLCPP_INFO(rclcpp::get_logger("ServoDriver"), "Response Length: %zu", resp_len);
        // 打印原始响应数据 (使用循环把每个字节以int形式加入字符串，最后打印，形如[1, 4, 0, 7, 101, 142])
        // std::ostringstream oss;
        // for (size_t i = 0; i < resp_len; ++i) {
        //     oss << static_cast<int>(resp[i]);
        //     if (i < resp_len - 1) oss << ", ";
        // }
        // RCLCPP_INFO(rclcpp::get_logger("ServoDriver"), "Raw Response: [%s]", oss.str().c_str());

        // --- 核心修改：检查最小长度 ---
        if (resp_len < 5) { // 如果舵机响应不包含 ID/LEN/CMD，只有 DATA(2) + CHK(1)，最小也得是 3 字节。
                            // 但根据您的解析 (resp[3] 和 resp[4])，至少需要 5 字节。
            RCLCPP_WARN(rclcpp::get_logger("ServoDriver"), "Response too short, skipping parsing.");
            return last_angle_;
        }
        
        // 假设 resp[3] 和 resp[4] 包含位置数据 (2字节)
        int pos = (resp[3] << 8) | resp[4];
        // RCLCPP_INFO(rclcpp::get_logger("ServoDriver"), "Parsed Position (0-4096): %d", pos);

        last_angle_ = positionToDegree(pos);
        return last_angle_;
    }
    
    RCLCPP_ERROR(rclcpp::get_logger("ServoDriver"), "sendCommand failed or resp_len is 0.");
    return last_angle_; // 通信失败，返回旧值
}

double Servo::currentDegree() {
    return getAngle();
}

int Servo::degreeToPosition(double degree) {
    degree = std::max(std::min(degree, SERVO_MAX_DEGREE), SERVO_MIN_DEGREE);
    double pos_range = SERVO_MAX_POSITION - SERVO_MIN_POSITION;
    double deg_range = SERVO_MAX_DEGREE - SERVO_MIN_DEGREE;
    double position = SERVO_MIN_POSITION + ((degree - SERVO_MIN_DEGREE) / deg_range) * pos_range;
    return static_cast<int>(position);
}

double Servo::positionToDegree(int position) {
    position = std::max(std::min(position, SERVO_MAX_POSITION), SERVO_MIN_POSITION);
    double pos_range = SERVO_MAX_POSITION - SERVO_MIN_POSITION;
    double deg_range = SERVO_MAX_DEGREE - SERVO_MIN_DEGREE;
    double degree = SERVO_MIN_DEGREE + ((position - SERVO_MIN_POSITION) / pos_range) * deg_range;
    return degree;
}
