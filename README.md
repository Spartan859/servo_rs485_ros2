# servo_rs485_ros2

## 简介
本包用于通过ROS2控制RS485总线舵机，支持ping、设置角度、读取角度等功能。

## 安装与编译
```bash
cd ~/ros2_ws/src
git clone <本仓库地址>
cd ..
colcon build
source install/setup.bash
```

## 启动节点
```bash
ros2 run servo_rs485_ros2 servo_rs485_node
```

## Service接口
- `/ping_servo`：检测舵机在线状态
- `/set_angle`：设置舵机角度
- `/get_angle`：获取当前角度

## Topic
- `/servo_angle`：周期发布当前舵机角度

## 参数
- `port`：串口设备名
- `servo_id`：舵机ID
- `baudrate`：波特率
- `timeout`：串口超时

## 示例调用
```bash
ros2 service call /set_angle servo_rs485_ros2/srv/SetAngle "{degree: 10.0, time_ms: 500}"
ros2 service call /get_angle servo_rs485_ros2/srv/GetAngle "{}"
```
