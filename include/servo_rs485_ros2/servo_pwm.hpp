#pragma once
#include "servo_rs485_ros2/servo_base.hpp"
#include <string>
#include <atomic>
#include <mutex>
#include <cstdint>

/**
 * @brief PWM 舵机实现（通过 devmem 操作寄存器）
 * 
 * 使用 devmem 命令直接操作硬件 PWM 寄存器来控制舵机角度。
 * 适用于直接连接到 SoC PWM 引脚的舵机。
 * 
 * 寄存器地址（默认值，可通过参数配置）：
 * - PWM 引脚复用：0x00C40000D0
 * - PWM 周期：0x00C408002C
 * - PWM 低电平开始：0x00C4080030
 * - PWM 高电平开始（占空比）：0x00C4080034
 */
class ServoPWM : public ServoBase {
public:
    /**
     * @brief 构造 PWM 舵机实例
     * @param pwm_period PWM 周期寄存器值（默认 10000）
     * @param duty_min 最小占空比值，对应 SERVO_MIN_DEGREE（默认 250，即 0.5ms/20ms）
     * @param duty_max 最大占空比值，对应 SERVO_MAX_DEGREE（默认 1250，即 2.5ms/20ms）
     * @param reg_mux_addr 引脚复用寄存器地址
     * @param reg_period_addr 周期寄存器地址
     * @param reg_low_addr 低电平开始寄存器地址
     * @param reg_high_addr 高电平开始寄存器地址（占空比控制）
     */
    ServoPWM(uint32_t pwm_period = 10000,
             uint32_t duty_min = 250,
             uint32_t duty_max = 1250,
             uint64_t reg_mux_addr = 0x00C40000D0,
             uint64_t reg_period_addr = 0x00C408002C,
             uint64_t reg_low_addr = 0x00C4080030,
             uint64_t reg_high_addr = 0x00C4080034);
    
    ~ServoPWM() override;

    bool ping() override;
    void setAngle(double degree, int time_ms = 0) override;
    void setAngleWithSpeed(double target_degree, double speed_dps = 30.0, 
                          int step_interval_ms = 50, std::atomic<bool>* cancel = nullptr) override;
    double getAngle() override;
    double currentDegree() override;
    std::string getType() const override { return "PWM"; }

private:
    // PWM 参数
    uint32_t pwm_period_;
    uint32_t duty_min_;
    uint32_t duty_max_;

    // 寄存器地址
    uint64_t reg_mux_addr_;
    uint64_t reg_period_addr_;
    uint64_t reg_low_addr_;
    uint64_t reg_high_addr_;

    // 状态
    bool initialized_ = false;
    mutable std::mutex io_mutex_;

    /**
     * @brief 初始化 PWM 硬件
     * @return true 初始化成功
     */
    bool initPWM();

    /**
     * @brief 通过 devmem 写入寄存器
     * @param addr 寄存器地址
     * @param value 写入的值
     * @return true 写入成功
     */
    bool writeRegister(uint64_t addr, uint32_t value);

    /**
     * @brief 通过 devmem 读取寄存器
     * @param addr 寄存器地址
     * @param value 读取的值（输出）
     * @return true 读取成功
     */
    bool readRegister(uint64_t addr, uint32_t& value);

    /**
     * @brief 将角度转换为 PWM 占空比值
     * @param degree 角度
     * @return PWM 占空比值
     */
    uint32_t degreeToDuty(double degree);

    /**
     * @brief 将 PWM 占空比值转换为角度
     * @param duty 占空比值
     * @return 角度
     */
    double dutyToDegree(uint32_t duty);
};
