#pragma once
#include <string>
#include <atomic>

/**
 * @brief 舵机抽象基类，定义通用接口
 * 
 * 子类实现：
 * - ServoRS485: RS485 总线舵机
 * - ServoPWM: PWM 舵机（通过 devmem 操作寄存器）
 */
class ServoBase {
public:
    virtual ~ServoBase() = default;

    /**
     * @brief 检测舵机是否在线
     * @return true 舵机在线
     */
    virtual bool ping() = 0;

    /**
     * @brief 设置舵机角度
     * @param degree 目标角度（度）
     * @param time_ms 运动时间（毫秒），某些舵机类型可能忽略此参数
     */
    virtual void setAngle(double degree, int time_ms = 0) = 0;

    /**
     * @brief 以指定速度平滑旋转到目标角度
     * @param target_degree 目标角度（度）
     * @param speed_dps 速度（度/秒），<=0 时直接跳转
     * @param step_interval_ms 步进间隔（毫秒）
     * @param cancel 取消标志，用于抢占
     */
    virtual void setAngleWithSpeed(double target_degree, double speed_dps = 30.0, 
                                   int step_interval_ms = 50, std::atomic<bool>* cancel = nullptr) = 0;

    /**
     * @brief 获取当前角度
     * @return 当前角度（度）
     */
    virtual double getAngle() = 0;

    /**
     * @brief 获取当前角度（别名）
     * @return 当前角度（度）
     */
    virtual double currentDegree() = 0;

    /**
     * @brief 获取舵机类型名称
     * @return 类型名称字符串
     */
    virtual std::string getType() const = 0;

protected:
    double last_angle_ = 0.0;

    // 角度范围常量
    static constexpr double SERVO_MIN_DEGREE = -135.0;
    static constexpr double SERVO_MAX_DEGREE = 135.0;

    /**
     * @brief 限制角度在有效范围内
     * @param degree 输入角度
     * @return 限制后的角度
     */
    double clampDegree(double degree) const {
        if (degree < SERVO_MIN_DEGREE) return SERVO_MIN_DEGREE;
        if (degree > SERVO_MAX_DEGREE) return SERVO_MAX_DEGREE;
        return degree;
    }
};
