# servo_rs485_ros2

## 简介
本包用于通过 ROS2 控制舵机，支持两种舵机类型：
- **RS485 总线舵机**：通过串口 RS485 协议控制
- **PWM 舵机**：通过 devmem 直接操作硬件 PWM 寄存器控制

支持功能：ping、设置角度、读取角度、匀速平滑旋转等。

## 安装与编译
```bash
cd ~/ros2_ws/src
git clone <本仓库地址>
cd ..
colcon build --packages-select servo_rs485_ros2
source install/setup.bash
```

## 启动节点

### RS485 舵机（默认）
```bash
ros2 run servo_rs485_ros2 servo_node --ros-args -p servo_type:=rs485 -p port:=/dev/ttyUSB1 -p servo_id:=1
```

### PWM 舵机
```bash
ros2 run servo_rs485_ros2 servo_node --ros-args \
  -p servo_type:=pwm \
  -p pwm_period:=10000 \
  -p pwm_duty_min:=250 \
  -p pwm_duty_max:=1250
```

## Service 接口
- `/ping_servo`：检测舵机在线状态
- `/set_angle`：设置舵机角度
- `/get_angle`：获取当前角度
- `/set_angle_with_speed`：以指定速度匀速旋转到目标角度

## Topic
- `/servo_angle`：周期发布当前舵机角度（std_msgs/Float64）

## 参数

### 通用参数
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `servo_type` | string | "rs485" | 舵机类型："rs485" 或 "pwm" |
| `publish_interval_ms` | int | 30 | 角度发布周期（毫秒） |

### RS485 舵机参数
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `port` | string | "/dev/ttyUSB1" | 串口设备路径 |
| `servo_id` | int | 1 | 舵机 ID |
| `baudrate` | int | 115200 | 波特率 |
| `timeout` | double | 0.5 | 串口超时（秒） |

### PWM 舵机参数
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `pwm_period` | int | 10000 | PWM 周期寄存器值 |
| `pwm_duty_min` | int | 250 | 最小占空比（对应 -135°） |
| `pwm_duty_max` | int | 1250 | 最大占空比（对应 +135°） |
| `pwm_reg_mux` | string | "0x00C40000D0" | 引脚复用寄存器地址 |
| `pwm_reg_period` | string | "0x00C408002C" | 周期寄存器地址 |
| `pwm_reg_low` | string | "0x00C4080030" | 低电平开始寄存器地址 |
| `pwm_reg_high` | string | "0x00C4080034" | 高电平开始寄存器地址（占空比） |

## 示例调用
```bash
# 设置角度：10 度，时长 500 ms
ros2 service call /set_angle servo_rs485_ros2/srv/SetAngle "{degree: 10.0, time_ms: 500}"

# 以 20 度/秒速度匀速旋转到 -15 度
ros2 service call /set_angle_with_speed servo_rs485_ros2/srv/SetAngleWithSpeed "{degree: -15.0, speed_dps: 20.0, step_interval_ms: 50}"

# 获取当前角度
ros2 service call /get_angle servo_rs485_ros2/srv/GetAngle "{}"

# 检测舵机在线
ros2 service call /ping_servo servo_rs485_ros2/srv/PingServo "{}"
```

## PWM 舵机使用说明

PWM 舵机通过 `devmem` 命令直接操作硬件寄存器。使用前需确保：

1. **权限**：运行节点的用户需要有执行 `devmem` 的权限（通常需要 root 或配置 udev 规则）
2. **寄存器地址**：根据实际硬件平台配置正确的寄存器地址
3. **占空比范围**：根据舵机规格调整 `pwm_duty_min` 和 `pwm_duty_max`

### PWM 占空比计算

对于典型的 20ms 周期舵机：
- 0.5ms 脉宽（-135°）：`0.5 / 20 * 10000 = 250`
- 1.5ms 脉宽（0°）：`1.5 / 20 * 10000 = 750`
- 2.5ms 脉宽（+135°）：`2.5 / 20 * 10000 = 1250`

### 手动测试 PWM
```bash
# 设置引脚复用
devmem 0x00C40000D0 w 0

# 设置周期
devmem 0x00C408002C w 10000

# 设置低电平开始
devmem 0x00C4080030 w 0

# 设置占空比（50% 示例）
devmem 0x00C4080034 w 5000

# 查询当前占空比
devmem 0x00C4080034
```
